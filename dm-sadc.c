#include <linux/device-mapper.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bio.h>
#include <linux/completion.h>
#include <linux/string_helpers.h>
#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/time.h>

//#include <stdio.h>
//#include <stdlib.h>
//#include <linux/time.h>

#define DM_MSG_PREFIX "sadc"

/* Disk state reset IOCTL command. */
#define RESET_DISK 0xDEADBEEF

#define pbas_in_bio          bio_segments

#define bio_begin_lba(bio)   ((bio)->bi_iter.bi_sector)
#define bio_end_lba          bio_end_sector

#define bio_begin_pba(bio)   (lba_to_pba(bio_begin_lba(bio)))
#define bio_end_pba(bio)     (lba_to_pba(bio_end_lba(bio)))

#define MIN_DISK_SIZE (76LL << 10)
#define MAX_DISK_SIZE (10LL << 40)

#define LBA_SIZE 512
#define PBA_SIZE 4096
#define LBAS_IN_PBA (PBA_SIZE / LBA_SIZE)

#define lba_to_pba(lba) ((pba_t) (lba / LBAS_IN_PBA))
#define pba_to_lba(pba) (((lba_t) pba) * LBAS_IN_PBA)

typedef sector_t lba_t;
typedef int32_t pba_t;

#define MIN_IOS 16
#define MIN_POOL_PAGES 32

#define RL_GC 1
// 模拟GC调度的情况
//动作数 idle\GC
#define ACTIONS 2
//探索次数 为1
#define episode 1
//目标状态 没有目标状态 null
#define target_state -1
//γ，折损率，取值是 0 到 1 之间。
#define gamma 8
//随机探索几率
#define epsilon 9
//学习率
#define alpha 1
#define short_max 32768
#define GC_threshold_percentage 99
#define GC_min_interval 100

// 需要对状态进行定义，如何更好地衡量系统的现状  【IO间隔、方差、上次采用的动作、persistent cache的剩余资源】
#define previous_interval_choices_max 2
#define current_interval_choices_max 5
#define previous_action_max 5
#define bands_left_max 4
#define pc_utilization_max 5
#define state_max previous_interval_choices_max*current_interval_choices_max*previous_action_max*bands_left_max*pc_utilization_max

//定义动作，便于查阅Q表
//代表要进行GC的band个数，其中0代表维持 当前状态不变
static int32_t action_list[5] = {0, 5, 10, 15, 20};
#define action_max 5

//定义状态空间
struct state{
    int32_t previous_interval;	// has 2 choices
    int32_t current_interval;  	// has 10 choices 暂定
    int32_t previous_action;	// 需要看动作空间如何定义，决定有几个候选项
    int32_t bands_left;
    int32_t pc_utilization;
};
static struct state current_state={0,0,0,0,0};
static int32_t current_action;
static int32_t reward;
static struct state next_state;
static int32_t next_state_max_q;
static int32_t last_io_begin_time=0;
static int32_t io_begin_time=0;
static int32_t io_end_time=0;
//定义Q表
static int32_t q[state_max][action_max] = { 0 };
//定义Q表的查询方式，即映射
static int32_t expore_count = 0;
static int32_t epsilon1 = 90;
static int32_t epsilon2 = 99;
static int32_t min_pba_in_cache_band = 256;
static int32_t init_complete = 0;



// struct pba_map_element{
//         pba_t data_band_pba;
//         pba_t cache_band_pba;
// };


static struct kmem_cache *_io_pool;

struct io {
        struct sadc_ctx *sc;
        struct bio *bio;
        struct work_struct work;
        atomic_t pending;
};

struct cache_band {
        int32_t nr;
        pba_t begin_pba;
        pba_t current_pba;

        unsigned long *map;
};

struct sadc_ctx {
        struct dm_dev *dev;
        int64_t disk_size;

        int32_t cache_percent;

        int64_t cache_size;
        int32_t nr_cache_bands;
        int64_t usable_size;
        int64_t wasted_size;
        int32_t track_size;
        int64_t band_size;
        int32_t band_size_tracks;
        int32_t band_size_pbas;
        int32_t nr_valid_pbas;
        int32_t nr_usable_pbas;

        pba_t *pba_map;
        // temp pba_map list
        //struct pba_map_element* tmp_pba_map_list;

        //维护逻辑cache band与真实cache band之间的映射关系
        int32_t *cache_bands_map;
        struct cache_band *cache_bands;

        //当前需要进行partialGC的band的index
        int32_t partialGC_band_index;
        //表示当前是否处于partialGC过程
        int32_t partialGC;
        //定义当前partialGC过程总共需要执行的band列表
        int32_t* partialGC_band_list;
        //partialGC过程当前执行到的band列表索引
        int32_t current_partialGC_band_list_index;
        int32_t current_partialGC_band_list_max;

        int32_t nr_bands;
        int32_t nr_usable_bands;
        int32_t cache_assoc;

        mempool_t *io_pool;
        mempool_t *page_pool;
        struct workqueue_struct *queue;
        struct mutex lock;
        struct completion io_completion;
        struct bio_set *bs;
        atomic_t error;
        struct bio **tmp_bios;
        struct bio **rmw_bios;
};
static int32_t get_row_q(struct state* s){
    int32_t idx = s->previous_interval*(current_interval_choices_max*previous_action_max*bands_left_max*pc_utilization_max)+\
                s->current_interval*(previous_action_max*bands_left_max*pc_utilization_max)+\
                s->previous_action*(bands_left_max*pc_utilization_max)+\
                s->bands_left*pc_utilization_max+\
                s->pc_utilization;
    return idx;
}
static int32_t is_empty_q(struct state* s){
    int32_t flag = 1;
    int32_t row = get_row_q(s);
        int32_t i = 0;
    for(;i<action_max;i++){
        if(q[row][i]!=0)
            flag = 0;
    }
    return flag;
}
//获取下个状态下最大的Q值
static int32_t max_value_q(struct state* s){
    int32_t i=1;
    int32_t row = get_row_q(s);
    int32_t max = q[row][0];
    for(;i<action_max;i++){
        if(q[row][i]>max)
            max = q[row][i];
    }
    return max;
}

static int32_t max_action(struct state* s){
    int32_t i=1,a=0;
    int32_t row = get_row_q(s);
    float max = q[row][0];
    for(;i<action_max;i++){
        if(q[row][i]>max)
            max = q[row][i];
            a = i;
    }
    return a;
}
//定义reward函数，这里最优的设计应分析目标数据流，但是这不现实
//可以根据经验进行定义，但是如何说明合理性有待讨论
static int32_t get_reward(int32_t response_time){
    int32_t r = 0;
    if(response_time<200){
            r = 100;
    }else if(response_time<1000){
            r =  50;
    }else if(response_time<3000){
            r = 0;
    }else{
            r = -100;
    }
    return r;
}
static int32_t current_interval_bins[current_interval_choices_max]=\
        {300, 1000, 3000, 10000, 30000};
static int32_t calc_current_interval(int32_t c){
    int32_t i = 0;
    for(;i<current_interval_choices_max;i++){
            if(c<current_interval_bins[i]){
                    break;
            }
    }
    return i;
}
static int get_bands_left(int b){
        int r = b/20;
        if(r<3){
                return r;
        }else{
                return 3;
        }
}


static int get_pc_utilization(int min){
    if(min>min_pba_in_cache_band){
        return 4;
    }
    return (min*pc_utilization_max)/min_pba_in_cache_band;

}
static struct state get_next_state(struct state* s, int32_t _current_action, int32_t b, int min){
    struct state ns;
    //PC利用率需要进行统计获取
    //ns.pc_utilization = 1;
    //当前IO时间与上次IO时间戳差值
    ns.current_interval = calc_current_interval(io_begin_time-last_io_begin_time);
    if(s->current_interval < 2){
            ns.previous_interval = 0;
    }else{
            ns.previous_interval = 1;
    }
    ns.previous_action = _current_action;
    ns.bands_left = get_bands_left(b);
    ns.pc_utilization = get_pc_utilization(min);
    return ns;
}

static char *readable(u64 size)
{
        static char buf[10];
        //void string_get_size(u64 size, u64 blk_size, enum string_size_units units, char *buf, int32_t len);
        string_get_size(size, 1,STRING_UNITS_2, buf, sizeof(buf));

        return buf;
}

static inline void debug_bio(struct sadc_ctx *sc, struct bio *bio, const char *f)

{
        int32_t i;
        unsigned long flags;
        struct bio_vec bv;

        pr_debug("%10s: %c offset: %d size: %u\n",
                 f,
                 (bio_data_dir(bio) == READ ? 'R' : 'W'),
                 bio_begin_pba(bio),
                 bio->bi_iter.bi_size);

        bio_for_each_segment(bv, bio, (bio)->bi_iter) {
                char *addr = bvec_kmap_irq(&bv, &flags);
                pr_debug("seg: %d, addr: %p, len: %u, offset: %u, char: [%d]\n",
                         i, addr, bv.bv_len, bv.bv_offset, *addr);
                bvec_kunmap_irq(addr, &flags);
        }
}

static bool unaligned_bio(struct bio *bio)
{
        return bio_begin_lba(bio) & 0x7 || bio->bi_iter.bi_size & 0xfff;
}

static struct io *alloc_io(struct sadc_ctx *sc, struct bio *bio)
{
        struct io *io = mempool_alloc(sc->io_pool, GFP_NOIO);

        if (unlikely(!io)) {
                DMERR("Could not allocate io from mempool!");
                return NULL;
        }

        memset(io, 0, sizeof(*io));

        io->sc = sc;
        io->bio = bio;

        atomic_set(&io->pending, 0);

        return io;
}

static void sadcd(struct work_struct *work);

static void queue_io(struct io *io)
{
        struct sadc_ctx *sc = io->sc;
        struct timeval ct;

        INIT_WORK(&io->work, sadcd);
        last_io_begin_time = io_begin_time;
        do_gettimeofday(&ct);
        io_begin_time = ct.tv_sec*1000+ct.tv_usec/1000;
        queue_work(sc->queue, &io->work);
}

static void release_io(struct io *io, int32_t error)
{
        struct sadc_ctx *sc = io->sc;
        bool rmw_bio = io->bio == NULL;

        WARN_ON(atomic_read(&io->pending));

        mempool_free(io, sc->io_pool);

        if (rmw_bio)
                atomic_set(&sc->error, error);
        else
                bio_endio(io->bio);
}

static inline bool usable_pba(struct sadc_ctx *sc, pba_t pba)
{
        return 0 <= pba && pba < sc->nr_usable_pbas;
}

static inline bool usable_band(struct sadc_ctx *sc, int32_t band)
{
        return 0 <= band && band < sc->nr_usable_bands;
}

static inline pba_t band_begin_pba(struct sadc_ctx *sc, int32_t band)
{
        WARN_ON(!usable_band(sc, band));

        return band * sc->band_size_pbas;
}

static inline pba_t band_end_pba(struct sadc_ctx *sc, int32_t band)
{
        return band_begin_pba(sc, band) + sc->band_size_pbas;
}

static inline int32_t pba_band(struct sadc_ctx *sc, pba_t pba)
{
        WARN_ON(!usable_pba(sc, pba));

        return pba / sc->band_size_pbas;
}

static inline int32_t bio_band(struct sadc_ctx *sc, struct bio *bio)
{
        return pba_band(sc, bio_begin_pba(bio));
}

static inline int32_t band_to_bit(struct sadc_ctx *sc, struct cache_band *cb,
                              int32_t band)
{
        return (band - cb->nr) / (sc->nr_cache_bands-1);
}

static inline int32_t bit_to_band(struct sadc_ctx *sc, struct cache_band *cb,
                              int32_t bit)
{
        return bit * (sc->nr_cache_bands-1) + cb->nr;
}
// static inline int32_t cache_band_map(struct sadc_ctx *sc, int32_t cb){
//         int32_t i;
//         for(i=0;i<sc->sc->nr_cache_bands;i++){
//                 if(sc->cache_bands_map[i]==cb){
//                         return i;
//                 }
//         }
// }
// cache band 的计算方式，这里是 % 
static inline struct cache_band *cache_band(struct sadc_ctx *sc, int32_t band)
{
        WARN_ON(!usable_band(sc, band));
        return &sc->cache_bands[sc->cache_bands_map[band % (sc->nr_cache_bands-1)]];
}

static inline int32_t free_pbas_in_cache_band(struct sadc_ctx *sc,
                                              struct cache_band *cb)
{
        return sc->band_size_pbas - (cb->current_pba - cb->begin_pba);
}

static int32_t pbas_in_band(struct sadc_ctx *sc, struct bio *bio, int32_t band)
{
        pba_t begin_pba = max(band_begin_pba(sc, band), bio_begin_pba(bio));
        pba_t end_pba = min(band_end_pba(sc, band), bio_end_pba(bio));

        return max(end_pba - begin_pba, 0);
}

static void unmap_pba_range(struct sadc_ctx *sc, pba_t begin, pba_t end)
{
        int32_t i;

        WARN_ON(begin >= end);
        WARN_ON(!usable_pba(sc, end - 1));

        for (i = begin; i < end; ++i)
                sc->pba_map[i] = -1;
}
// 将一段数据置入到PC中，建立映射关系
static pba_t map_pba_range(struct sadc_ctx *sc, pba_t begin, pba_t end)
{
        pba_t i;
        int32_t b;
        struct cache_band *cb;

        WARN_ON(begin >= end);
        WARN_ON(!usable_pba(sc, end - 1));

        b = pba_band(sc, begin);

        WARN_ON(b != pba_band(sc, end - 1));

        cb = cache_band(sc, b);

        WARN_ON(free_pbas_in_cache_band(sc, cb) < (end - begin));

        for (i = begin; i < end; ++i)
                sc->pba_map[i] = cb->current_pba++;

        set_bit(band_to_bit(sc, cb, b), cb->map);

        return sc->pba_map[begin];
}
// 查看当前地址在PC中还是在原位置
static inline pba_t lookup_pba(struct sadc_ctx *sc, pba_t pba)
{
        WARN_ON(!usable_pba(sc, pba));

        return sc->pba_map[pba] == -1 ? pba : sc->pba_map[pba];
}

static inline lba_t lookup_lba(struct sadc_ctx *sc, lba_t lba)
{
        return pba_to_lba(lookup_pba(sc, lba_to_pba(lba))) + lba % LBAS_IN_PBA;
}

static void do_free_bio_pages(struct sadc_ctx *sc, struct bio *bio)
{
        int32_t i;
        struct bio_vec *bv;

        bio_for_each_segment_all(bv, bio, i) {
                WARN_ON(!bv->bv_page);
                mempool_free(bv->bv_page, sc->page_pool);
                bv->bv_page = NULL;
        }

        /* For now we should only have a single page per bio. */
        WARN_ON(i != 1);
}

static void endio(struct bio *bio)
{
        struct io *io = bio->bi_private;
        struct sadc_ctx *sc = io->sc;
        bool rmw_bio = io->bio == NULL;

        if (rmw_bio && bio_data_dir(bio) == WRITE)
                do_free_bio_pages(sc, bio);

        bio_put(bio);

        if (atomic_dec_and_test(&io->pending)) {
                release_io(io, 0);
                complete(&sc->io_completion);
        }
}
// pda地址是否连续
static bool adjacent_pbas(struct sadc_ctx *sc, pba_t x, pba_t y)
{
        return lookup_pba(sc, x) + 1 == lookup_pba(sc, y);
}

static struct bio *clone_remap_bio(struct io *io, struct bio *bio, int32_t idx,
                                   pba_t pba, int32_t nr_pbas)
{
        struct sadc_ctx *sc = io->sc;
        struct bio *clone;

        clone = bio_clone_bioset(bio, GFP_NOIO, sc->bs);
        if (unlikely(!clone)) {
                DMERR("Cannot clone a bio.");
                return NULL;
        }

        if (bio_data_dir(bio) == READ)
                pba = lookup_pba(sc, pba);
        else
                pba = map_pba_range(sc, pba, pba + nr_pbas);

        clone->bi_iter.bi_sector = pba_to_lba(pba);
        clone->bi_private = io;
        clone->bi_end_io = endio;
        clone->bi_bdev = sc->dev->bdev;

        clone->bi_iter.bi_idx = idx;
        clone->bi_vcnt = idx + nr_pbas;
        clone->bi_iter.bi_size = nr_pbas * PBA_SIZE;

        atomic_inc(&io->pending);

        return clone;
}

static void release_bio(struct bio *bio)
{
        struct io *io = bio->bi_private;

        atomic_dec(&io->pending);
        bio_put(bio);
}

static int32_t handle_unaligned_io(struct sadc_ctx *sc, struct io *io)
{
        struct bio *bio = io->bio;
        struct bio *clone = bio_clone_bioset(bio, GFP_NOIO, sc->bs);

        WARN_ON(bio_data_dir(bio) != READ);
        WARN_ON(bio_end_lba(bio) > pba_to_lba(bio_begin_pba(bio) + 1));

        if (unlikely(!clone)) {
                DMERR("Cannot clone a bio.");
                return -ENOMEM;
        }

        clone->bi_iter.bi_sector = lookup_lba(sc, bio_begin_lba(bio));
        clone->bi_private = io;
        clone->bi_end_io = endio;
        clone->bi_bdev = sc->dev->bdev;

        atomic_inc(&io->pending);

        sc->tmp_bios[0] = clone;

        return 1;
}

static int32_t split_read_io(struct sadc_ctx *sc, struct io *io)
{
        struct bio *bio = io->bio;
        pba_t bp, p;
        int32_t i, n = 0, idx = 0;

        if (unlikely(unaligned_bio(bio)))
                return handle_unaligned_io(sc, io);

        bp = bio_begin_pba(bio);
        p = bp + 1;

        for (i = 1; p < bio_end_pba(bio); ++i, ++p) {
                if (adjacent_pbas(sc, p - 1, p))
                        continue;
                sc->tmp_bios[n] = clone_remap_bio(io, bio, idx, bp, i - idx);
                if (!sc->tmp_bios[n])
                        goto bad;
                ++n, idx = i, bp = p;
        }

        sc->tmp_bios[n] = clone_remap_bio(io, bio, idx, bp, i - idx);
        if (sc->tmp_bios[n])
                return n + 1;

bad:
        while (n--)
                release_bio(sc->tmp_bios[n]);
        return -ENOMEM;
}

static int32_t split_write_io(struct sadc_ctx *sc, struct io *io)
{
        struct bio *bio = io->bio;
        int32_t nr_pbas_bio1, nr_pbas_bio2;
        int32_t idx = 0;
        pba_t p;

        nr_pbas_bio1 = pbas_in_band(sc, bio, bio_band(sc, bio));
        nr_pbas_bio2 = pbas_in_bio(bio) - nr_pbas_bio1;
        p = bio_begin_pba(bio);

        sc->tmp_bios[0] = clone_remap_bio(io, bio, idx, p, nr_pbas_bio1);
        if (!sc->tmp_bios[0])
                return -ENOMEM;

        if (!nr_pbas_bio2)
                return 1;

        p += nr_pbas_bio1;
        idx += nr_pbas_bio1;

        sc->tmp_bios[1] =
                clone_remap_bio(io, bio, idx, p, nr_pbas_bio2);
        if (!sc->tmp_bios[1]) {
                release_bio(sc->tmp_bios[0]);
                return -ENOMEM;
        }

        return 2;
}

static int32_t do_sync_io(struct sadc_ctx *sc, struct bio **bios, int32_t n)
{
        int32_t i;
        if(bios[0]==NULL){
                return -1;
        }
        reinit_completion(&sc->io_completion);

        for (i = 0; i < n; ++i)
                generic_make_request(bios[i]);

        wait_for_completion(&sc->io_completion);

        return atomic_read(&sc->error);
}

typedef int32_t (*split_t)(struct sadc_ctx *sc, struct io *io);

static void do_io(struct sadc_ctx *sc, struct io *io, split_t split)
{
        int32_t n = split(sc, io);

        if (n < 0) {
                release_io(io, n);
                return;
        }

        WARN_ON(!n);

        do_sync_io(sc, sc->tmp_bios, n);
}

static struct cache_band *cache_band_to_gc(struct sadc_ctx *sc, struct bio *bio)
{
        int32_t b = bio_band(sc, bio);
        int32_t nr_pbas = pbas_in_band(sc, bio, b);
        struct cache_band *cb = cache_band(sc, b);

        if (free_pbas_in_cache_band(sc, cb) < nr_pbas)
                return cb;

        if (!usable_band(sc, ++b))
                return NULL;

        cb = cache_band(sc, b);
        nr_pbas = pbas_in_bio(bio) - nr_pbas;

        return free_pbas_in_cache_band(sc, cb) < nr_pbas ? cb : NULL;
}

static struct bio *alloc_bio_with_page(struct sadc_ctx *sc, pba_t pba)
{
        struct page *page = mempool_alloc(sc->page_pool, GFP_NOIO);
        struct bio *bio = bio_alloc_bioset(GFP_NOIO, 1, sc->bs);

        if (!bio || !page)
                goto bad;

        bio->bi_iter.bi_sector = pba_to_lba(pba);
        bio->bi_bdev = sc->dev->bdev;

        if (!bio_add_page(bio, page, PAGE_SIZE, 0))
                goto bad;

        return bio;

bad:
        if (page)
                mempool_free(page, sc->page_pool);
        if (bio)
                bio_put(bio);
        return NULL;
}

static void free_rmw_bios(struct sadc_ctx *sc, int32_t n)
{
        int32_t i;

        for (i = 0; i < n; ++i) {
                do_free_bio_pages(sc, sc->rmw_bios[i]);
                bio_put(sc->rmw_bios[i]);
        }
}

static bool alloc_rmw_bios(struct sadc_ctx *sc, int32_t band)
{
        pba_t p = band_begin_pba(sc, band);
        int32_t i;

        for (i = 0; i < sc->band_size_pbas; ++i) {
                sc->rmw_bios[i] = alloc_bio_with_page(sc, p + i);
                if (!sc->rmw_bios[i])
                        goto bad;
        }
        return true;

bad:
        free_rmw_bios(sc, i);
        return false;
}

static struct bio *clone_bio(struct io *io, struct bio *bio, pba_t pba)
{
        struct sadc_ctx *sc = io->sc;
        struct bio *clone = bio_clone_bioset(bio, GFP_NOIO, sc->bs);

        if (unlikely(!clone)) {
                DMERR("Cannot clone bio.");
                return NULL;
        }

        clone->bi_private = io;
        clone->bi_end_io = endio;
        clone->bi_iter.bi_sector = pba_to_lba(pba);

        atomic_inc(&io->pending);

        return clone;
}

static int32_t do_read_band(struct sadc_ctx *sc, int32_t band)
{
        struct io *io = alloc_io(sc, NULL);
        pba_t p = band_begin_pba(sc, band);
        int32_t i;

        if (unlikely(!io))
                return -ENOMEM;

        for (i = 0; i < sc->band_size_pbas; ++i) {
                sc->tmp_bios[i] = clone_bio(io, sc->rmw_bios[i], p + i);
                if (!sc->tmp_bios[i])
                        goto bad;
        }

        return do_sync_io(sc, sc->tmp_bios, sc->band_size_pbas);

bad:
        while (i--)
                release_bio(sc->tmp_bios[i]);
        return -ENOMEM;
}

static int32_t do_modify_band(struct sadc_ctx *sc, int32_t band)
{
        struct io *io = alloc_io(sc, NULL);
        pba_t p = band_begin_pba(sc, band);
        int32_t i, j;

        if (unlikely(!io))
                return -ENOMEM;

        for (i = j = 0; i < sc->band_size_pbas; ++i) {
                pba_t pp = lookup_pba(sc, bio_begin_pba(sc->rmw_bios[i]));

                if (pp == p + i)
                        continue;

                sc->tmp_bios[j] = clone_bio(io, sc->rmw_bios[i], pp);
                if (!sc->tmp_bios[j])
                        goto bad;
                ++j;
        }
        if(!j){
                printk("do_modify_band: %d j=0", band);
                return;
        }
        //WARN_ON(!j);

        return do_sync_io(sc, sc->tmp_bios, j);

bad:
        while (j--)
                release_bio(sc->tmp_bios[j]);
        return -ENOMEM;
}

static int32_t do_write_band(struct sadc_ctx *sc, int32_t band)
{
        struct io *io = alloc_io(sc, NULL);
        int32_t i;

        if (unlikely(!io))
                return -ENOMEM;

        for (i = 0; i < sc->band_size_pbas; ++i) {
                sc->rmw_bios[i]->bi_private = io;
                sc->rmw_bios[i]->bi_end_io = endio;
                sc->rmw_bios[i]->bi_opf = WRITE;
        }

        atomic_set(&io->pending, sc->band_size_pbas);

        return do_sync_io(sc, sc->rmw_bios, sc->band_size_pbas);
}

static int32_t do_rmw_band(struct sadc_ctx *sc, int32_t band)
{
        int32_t r = 0;

        if (!alloc_rmw_bios(sc, band))
                return -ENOMEM;

        //DMINFO("Reading band %d\n", band);
        r = do_read_band(sc, band);
        if (r < 0)
                goto bad;

        //DMINFO("Modifying band %d\n", band);
        r = do_modify_band(sc, band);
        if (r < 0)
                goto bad;

        //DMINFO("Writing band %d\n", band);
        return do_write_band(sc, band);

bad:
        printk("do_rmw_band bad!");
        free_rmw_bios(sc, sc->band_size_pbas);
        return r;
}

static void print_cb_info(struct sadc_ctx *sc){
        int32_t i;
        printk("Cache Band info:");
        for(i=0;i<sc->nr_cache_bands;i++){
                int32_t fpic = free_pbas_in_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[i]]);
                printk("Cache Band: %d, %d, %p, %d", i, sc->cache_bands_map[i],&sc->cache_bands[sc->cache_bands_map[i]], fpic);
        }
}
static void reset_cache_band(struct sadc_ctx *sc, struct cache_band *cb)
{
        //printk("Before reset_cache_band :%p", cb);
        cb->current_pba = cb->begin_pba;
        // if(init_complete)
        //         print_cb_info(sc);
        //printk("Mid reset_cache_band");
        bitmap_zero(cb->map, sc->cache_assoc);
        //printk("After reset_cache_band");
}

// static int32_t do_gc_cache_band(struct sadc_ctx *sc, struct cache_band *cb)
// {
//         int32_t i;

//         for_each_set_bit(i, cb->map, sc->cache_assoc) {
//                 int32_t b = bit_to_band(sc, cb, i);
//                 int32_t r = do_rmw_band(sc, b);
//                 if (r < 0)
//                         return r;
//                 unmap_pba_range(sc, band_begin_pba(sc, b), band_end_pba(sc, b));
//         }
//         reset_cache_band(sc, cb);
//         return 0;
// }
//真正执行partialGC过程的数据band
static int32_t do_partialGC_every_band(struct sadc_ctx *sc, int32_t b)
{
        int32_t r = do_rmw_band(sc, b);
        //printk("sadc do_partialGC_every_band!!!");
        if (r < 0)
                return r;
        unmap_pba_range(sc, band_begin_pba(sc, b), band_end_pba(sc, b));
        return 0;
}
//设置一个函数，取得所有要GC的band列表
static void get_partialGC_band_list(struct sadc_ctx *sc)
{
        int32_t i,index=0;
        struct cache_band *cb = &sc->cache_bands[sc->cache_bands_map[sc->partialGC_band_index]];
        //list设置为要做GC的band
        for_each_set_bit(i, cb->map, sc->cache_assoc) {
                int32_t b = bit_to_band(sc, cb, i);
                sc->partialGC_band_list[index] = b;
                index+=1;
        }
        //将list后面的全部置-1
        for(;index<sc->cache_assoc;index++){
                sc->partialGC_band_list[index] = -1;
        }
        // printk("Band list:");
        // for(index=0;index<sc->cache_assoc;index++){
        //     if(sc->partialGC_band_list[index]==-1){
        //         break;
        //     }
        //     printk("%d", sc->partialGC_band_list[index]);
        // }
        sc->current_partialGC_band_list_max = index;
}
//当前partialGC过程结束，需要重置相关的值
static void end_of_partialGC(struct sadc_ctx *sc){
        printk("sadc: end_of_partialGC");
        sc->partialGC = 0;
        sc->current_partialGC_band_list_index = 0;
        sc->current_partialGC_band_list_max = 0;
        //重置相关cb
        reset_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[sc->partialGC_band_index]]);
}
static void do_partialGC_to_end(struct sadc_ctx *sc)
{
        int32_t base = sc->current_partialGC_band_list_index,i;
        DMINFO("sadc: do_partialGC_to_end Start.\n");
        for(i=0;i<sc->nr_usable_bands;i++){
                //若到达partialGC_band_list的结尾
                if(sc->partialGC_band_list[base+i]==-1){
                        goto end;
                }
                //printk("sadc: do partialGC every band: %d, %d", base+i, sc->partialGC_band_list[base+i]);
                do_partialGC_every_band(sc, sc->partialGC_band_list[base+i]);
        }
        return;
end:
        end_of_partialGC(sc);
        DMINFO("sadc: do_partialGC_to_end Complete.\n");
}
static int32_t do_gc_cache_band(struct sadc_ctx *sc, struct cache_band *cb)
{
        int32_t i;

        for_each_set_bit(i, cb->map, sc->cache_assoc) {
                int32_t b = bit_to_band(sc, cb, i);
                int32_t r = do_rmw_band(sc, b);
                if (r < 0)
                        return r;
                unmap_pba_range(sc, band_begin_pba(sc, b), band_end_pba(sc, b));
        }
        reset_cache_band(sc, cb);
        DMINFO("sadc: do_gc_cache_band.\n");
        return 0;
}

static void next_partialGC(struct sadc_ctx *sc, struct bio *bio)
{
        int32_t target_cb_index,t,i=0;
        int32_t b = bio_band(sc, bio);
        int32_t nr_pbas = pbas_in_band(sc, bio, b);
        struct cache_band *cb = cache_band(sc, b);

        // need GC right now
        if (free_pbas_in_cache_band(sc, cb) < nr_pbas){
                target_cb_index = b % (sc->nr_cache_bands-1);
                printk("sadc: next Start_partialGC!");
                //映射表中，需交换位置
                //printk("1");
                reset_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[sc->partialGC_band_index]]);
                //printk("partialGC_band_index: %d", sc->partialGC_band_index);
                t = sc->cache_bands_map[sc->partialGC_band_index];
                sc->cache_bands_map[sc->partialGC_band_index] = sc->cache_bands_map[target_cb_index];
                sc->cache_bands_map[target_cb_index] = t;
                sc->cache_bands[sc->cache_bands_map[target_cb_index]].nr = target_cb_index;
                sc->partialGC = 1;
                //printk("2");
                get_partialGC_band_list(sc);
                //printk("3");
                sc->current_partialGC_band_list_index = 0;
                if(init_complete)
                        print_cb_info(sc);
                printk("sadc: end of next Start_partialGC!");
        }
}
static int32_t do_gc_if_required(struct sadc_ctx *sc, struct bio *bio)
{
        //如果处于partialGC过程，需要马上完成当前的partialGC过程，然后执行完整的GC过程

        //首先检查是否有需要GC的cb
        struct cache_band *cb;
        int32_t r;
        //printk("sadc: do_gc_if_required!");
        cb = cache_band_to_gc(sc, bio);
        r = 0;
        if (!cb)
                return r;
        printk("do gc if required 1: %p", cb);
        if(sc->partialGC){
                do_partialGC_to_end(sc);
        }
        next_partialGC(sc,bio);
        //重新检查是否还有需要GC的cb
        // cb = cache_band_to_gc(sc, bio);
        // printk("do gc if required 2: %p", cb);
        // if(init_complete)
        //         print_cb_info(sc);
        // r = 0;
        // if (!cb)
        //         return r;
        // DMINFO("sadc: Starting GC.\n");
        // do {
        //         r = do_gc_cache_band(sc, cb);
        //         if (r < 0)
        //                 return r;
        //         cb = cache_band_to_gc(sc, bio);
        //         printk("do gc if required 3: %p", cb);
        // } while (cb);

        // DMINFO("sadc: GC completed.\n");
        return r;  
}
// 挑选应该做GC的目标cb
static void start_partialGC(struct sadc_ctx *sc)
{
        int32_t t, i=0;
        printk("sadc: Start_partialGC!");
        int32_t cache_band_min = sc->band_size_pbas;
        int32_t cahce_band_min_idx = 0;
        for(;i<sc->nr_cache_bands-1;i++){
                int32_t fpic = free_pbas_in_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[i]]);
                if(fpic<cache_band_min){
                        cache_band_min = fpic;
                        cahce_band_min_idx = i;
                }
        }
        //映射表中，需交换位置
        //printk("1");
        reset_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[sc->partialGC_band_index]]);
        //printk("partialGC_band_index: %d", sc->partialGC_band_index);
        t = sc->cache_bands_map[sc->partialGC_band_index];
        sc->cache_bands_map[sc->partialGC_band_index] = sc->cache_bands_map[cahce_band_min_idx];
        //sc->cache_bands[sc->cache_bands_map[sc->partialGC_band_index]].nr = cahce_band_min_idx;
        sc->cache_bands_map[cahce_band_min_idx] = t;
        sc->cache_bands[sc->cache_bands_map[cahce_band_min_idx]].nr = cahce_band_min_idx;
        sc->partialGC = 1;
        //printk("2");
        get_partialGC_band_list(sc);
        //printk("3");
        sc->current_partialGC_band_list_index = 0;
        printk("sadc: end of Start_partialGC!");
}

//进行partialGC，band_to_gc 代表当前可以执行的RMW的band数量
static void do_partialGC(struct sadc_ctx *sc, int32_t nr_band_to_gc)
{
        int32_t base, i;
        //维持当前状态不变
        if(nr_band_to_gc==0){
                return;
        }
        DMINFO("sadc: Starting partialGC, # of bands: %d.\n", nr_band_to_gc);
        //若当前不处于partialGC状态，则应开始一个partialGC过程
        if(!sc->partialGC){
                start_partialGC(sc);
        }
        if(init_complete)
                print_cb_info(sc);
        base = sc->current_partialGC_band_list_index;
        for(i=0;i<nr_band_to_gc;i++){
                //若到达partialGC_band_list的结尾
                if(sc->partialGC_band_list[base+i]==-1){
                        goto end;
                }
                //printk("sadc: do partialGC every band: %d, %d", base+i, sc->partialGC_band_list[base+i]);
                do_partialGC_every_band(sc, sc->partialGC_band_list[base+i]);
        }
        sc->current_partialGC_band_list_index += nr_band_to_gc;
        DMINFO("sadc: PartialGC complete.\n");
        return;
end:
        end_of_partialGC(sc);
        DMINFO("sadc: Current partialGC complete.\n");
}

void get_random_bytes(void *buf, int32_t nbytes);
static int32_t get_random_number(void)
{
    unsigned short randNum;
    get_random_bytes(&randNum, sizeof(unsigned short));
    return randNum;
}

static int32_t get_pc_used_min(struct sadc_ctx *sc){
        int32_t i=0, f=0, min = sc->band_size_pbas;
        int32_t flag = 0;
        for(;i<sc->nr_cache_bands-1;i++){
            f = free_pbas_in_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[i]]);
            //printk("free pbas in cache band %d: %d", i, f);
            if(f<min)
                min = f;
                    
        }
        return min;
}

static int32_t get_pc_used_percentage(struct sadc_ctx *sc){
        int32_t i=0, f=0;
        int32_t flag = 0;
        for(;i<sc->nr_cache_bands-1;i++){
            f = free_pbas_in_cache_band(sc, &sc->cache_bands[sc->cache_bands_map[i]]);
            //printk("free pbas in cache band %d: %d", i, f);
            if(f<min_pba_in_cache_band)
                return 1;
                    
        }
        return 0;
}
static void sadcd(struct work_struct *work)
{
    struct io* io = container_of(work, struct io, work);
    struct sadc_ctx* sc = io->sc;
    struct bio* bio = io->bio;
    struct timeval ct;
    int32_t row, randNum;

    mutex_lock(&sc->lock);

    if (bio_data_dir(bio) == READ) {
        do_io(sc, io, split_read_io);
    }
    else {
        int32_t r;
        WARN_ON(unaligned_bio(bio));
        /*
        这里的逻辑需要重新理一下
        若当前有需要进行GC的任意band，则必须立即结束partialGC过程，并进行必要的GC过程
        */
        r = do_gc_if_required(sc, bio);
        //printk("r: %d",r);
        if (r < 0){
            release_io(io, r);
        }
        else {
            do_io(sc, io, split_write_io);
            //printk("sadc: Do_io");
            //if (RL_GC && get_pc_used_percentage(sc) && (io_begin_time - last_io_begin_time > GC_min_interval)) {
            //if (RL_GC && (io_begin_time - last_io_begin_time > GC_min_interval)) {
            if (RL_GC && get_pc_used_percentage(sc)) {
                do_gettimeofday(&ct);
                io_end_time = ct.tv_sec * 1000 + ct.tv_usec / 1000; //ms
                randNum = get_random_number();
                if (expore_count < 3000) {
                    if ((((randNum + short_max) / (short_max * 2 - 1)) / 100 > epsilon1) || (is_empty_q(&current_state) == 1)) {
                        current_action = get_random_number() % (action_max);
                    }
                    else {
                        current_action = max_action(&current_state);
                    }
                }else {
                    if ((((randNum + short_max) / (short_max * 2 - 1)) / 100 > epsilon2) || (is_empty_q(&current_state) == 1)) {
                        current_action = get_random_number() % (action_max);
                    }
                    else {
                        current_action = max_action(&current_state);
                    }
                }
                expore_count += 1;

                //do_action() idle/GC
                printk("sadc: sadcd-before do_partialGC iteration: %d", expore_count);
                do_partialGC(sc, action_list[current_action]);
                // IO请求的响应时间对应reward
                reward = get_reward(io_end_time - io_begin_time);
                next_state = get_next_state(&current_state, current_action, \
                        sc->current_partialGC_band_list_max-sc->current_partialGC_band_list_index,\
                        get_pc_used_min(sc));
                // 获得下一个状态中，在自我经验中，也就是 Q 矩阵的最有价值的动作的经验得分。
                next_state_max_q = max_value_q(&next_state);
                // 当前动作的经验总得分 = 当前动作得分 + γ X 执行该动作后的下一个状态的最大的经验得分
                // 即：积累经验 = 动作执行后的即时奖励 + 下一状态根据现有学习经验中最有价值的选择 X 折扣率
                row = get_row_q(&current_state);
                q[row][current_action] = q[row][current_action] + alpha * (reward + gamma * next_state_max_q / 10 - q[row][current_action]) / 10;
                current_state = next_state;
            }
        }
    }
    mutex_unlock(&sc->lock);
}

static bool get_args(struct dm_target *ti, struct sadc_ctx *sc,
                     int32_t argc, char **argv)
{
        unsigned long long tmp;
        char d;

        if (argc != 5) {
                ti->error = "dm-sadc: Invalid argument count.";
                return false;
        }

        if (sscanf(argv[1], "%llu%c", &tmp, &d) != 1 || tmp & 0xfff ||
            (tmp < 4 * 1024 || tmp > 2 * 1024 * 1024)) {
                ti->error = "dm-sadc: Invalid track size.";
                return false;
        }
        sc->track_size = tmp;

        if (sscanf(argv[2], "%llu%c", &tmp, &d) != 1 || tmp < 1 || tmp > 200) {
                ti->error = "dm-sadc: Invalid band size.";
                return false;
        }
        sc->band_size_tracks = tmp;

        if (sscanf(argv[3], "%llu%c", &tmp, &d) != 1 || tmp < 1 || tmp > 50) {
                ti->error = "dm-sadc: Invalid cache percent.";
                return false;
        }
        sc->cache_percent = tmp;

        if (sscanf(argv[4], "%llu%c", &tmp, &d) != 1 ||
            tmp < MIN_DISK_SIZE || tmp > MAX_DISK_SIZE) {
                ti->error = "dm-sadc: Invalid disk size.";
                return false;
        }
        sc->disk_size = tmp;

        return true;
}

static void calc_params(struct sadc_ctx *sc)
{
        sc->band_size      = sc->band_size_tracks * sc->track_size;
        sc->band_size_pbas = sc->band_size / PBA_SIZE;
        sc->nr_bands       = sc->disk_size / sc->band_size;
        // 多出的一个作为temp band
        sc->nr_cache_bands = sc->nr_bands * sc->cache_percent / 100 + 1;
        sc->cache_size     = sc->nr_cache_bands * sc->band_size;

        /*
         * Make |nr_usable_bands| a multiple of |nr_cache_bands| so that all
         * cache bands are equally loaded.
         */
        //-1 是因为映射关系不应该包括temp band
        sc->nr_usable_bands  = (sc->nr_bands / (sc->nr_cache_bands-1) - 1) *
                (sc->nr_cache_bands-1);
        // 每个cache band关联的数据band个数
        sc->cache_assoc    = sc->nr_usable_bands / (sc->nr_cache_bands-1);
        sc->usable_size    = sc->nr_usable_bands * sc->band_size;
        sc->wasted_size    = sc->disk_size - sc->cache_size - sc->usable_size;
        sc->nr_valid_pbas  = (sc->usable_size + sc->cache_size) / PBA_SIZE;
        sc->nr_usable_pbas = sc->usable_size / PBA_SIZE;
        sc->partialGC_band_index = sc->nr_cache_bands-1;
        //当前不处于partialGC过程
        sc->partialGC = 0;

        WARN_ON(sc->usable_size % PBA_SIZE);
}

static void print_params(struct sadc_ctx *sc)
{
        DMINFO("Disk size: %s",              readable(sc->disk_size));
        DMINFO("Band size: %s",              readable(sc->band_size));
        DMINFO("Band size: %d pbas",         sc->band_size_pbas);
        DMINFO("Total number of bands: %d",  sc->nr_bands);
        DMINFO("Number of cache bands: %d",  sc->nr_cache_bands-1);
        DMINFO("Number of temp cache band: %d",  1);
        DMINFO("Cache size: %s",             readable(sc->cache_size));
        DMINFO("Number of usable bands: %d", sc->nr_usable_bands);
        DMINFO("Usable disk size: %s",       readable(sc->usable_size));
        DMINFO("Number of usable pbas: %d",  sc->nr_usable_pbas);
        DMINFO("Wasted disk size: %s",       readable(sc->wasted_size));
}

static void sadc_dtr(struct dm_target *ti)
{
        int32_t i;
        struct sadc_ctx *sc = (struct sadc_ctx *) ti->private;

        DMINFO("Destructing...");

        ti->private = NULL;

        if (!sc)
                return;

        if (sc->tmp_bios)
                vfree(sc->tmp_bios);
        if (sc->rmw_bios)
                vfree(sc->rmw_bios);
        if (sc->pba_map)
                vfree(sc->pba_map);
        if(sc->partialGC_band_list)
                vfree(sc->partialGC_band_list);
        if(sc->cache_bands_map)
                vfree(sc->cache_bands_map);

        for (i = 0; i < sc->nr_cache_bands; ++i)
                if (sc->cache_bands[i].map)
                        kfree(sc->cache_bands[i].map);

        if (sc->cache_bands)
                vfree(sc->cache_bands);

        if (sc->io_pool)
                mempool_destroy(sc->io_pool);
        if (sc->queue)
                destroy_workqueue(sc->queue);
        if (sc->dev)
                dm_put_device(ti, sc->dev);
        kzfree(sc);
}

static bool alloc_structs(struct sadc_ctx *sc)
{
        int32_t i, size, pba;

        size = sizeof(int32_t) * sc->nr_usable_pbas;
        sc->pba_map = vmalloc(size);
        if (!sc->pba_map)
                return false;
        memset(sc->pba_map, -1, size);

        //定义存储要GC的band的列表
        size = sizeof(int32_t) * sc->nr_usable_bands;
        sc->partialGC_band_list = vmalloc(size);
        if (!sc->partialGC_band_list)
                return false;
        memset(sc->partialGC_band_list, -1, size);

        size = sizeof(struct bio *) * sc->band_size_pbas;
        sc->rmw_bios = vzalloc(size);
        if (!sc->rmw_bios)
                return false;

        sc->tmp_bios = vzalloc(size);
        if (!sc->tmp_bios)
                return false;

        //申请cache_band_map所需的空间
        size = sizeof(int32_t) * sc->nr_cache_bands;
        sc->cache_bands_map = vmalloc(size);
        if (!sc->cache_bands_map)
                return false;
        for(i=0;i < sc->nr_cache_bands;i++){
                sc->cache_bands_map[i] = i;
        }

        size = sizeof(struct cache_band) * sc->nr_cache_bands;
        sc->cache_bands = vmalloc(size);
        if (!sc->cache_bands)
                return false;

        /* The cache region starts where the data region ends. */
        pba = sc->nr_usable_pbas;

        size = BITS_TO_LONGS(sc->cache_assoc) * sizeof(long);
        for (i = 0; i < sc->nr_cache_bands; ++i, pba += sc->band_size_pbas) {
                sc->cache_bands[i].nr = i;
                sc->cache_bands[i].begin_pba = pba;
                sc->cache_bands[i].map = kmalloc(size, GFP_KERNEL);
                if (!sc->cache_bands[i].map)
                        return false;
                reset_cache_band(sc, &sc->cache_bands[i]);
        }
        //temp的位置置-1
        //sc->cache_bands_map[sc->nr_cache_bands-1] = -1;
        init_complete = 1;
        return true;
}
//定义模块加载的函数
static int32_t sadc_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
        struct sadc_ctx *sc;
        int32_t ret;
        struct timeval ct;

        DMINFO("RL2----Constructing...");

        sc = kzalloc(sizeof(*sc), GFP_KERNEL);
        if (!sc) {
                ti->error = "dm-sadc: Cannot allocate sadc context.";
                return -ENOMEM;
        }
        ti->private = sc;

        if (!get_args(ti, sc, argc, argv)) {
                kzfree(sc);
                return -EINVAL;
        }

        calc_params(sc);
        print_params(sc);

        ret = -ENOMEM;
        if (!alloc_structs(sc)) {
            ti->error = "Cannot allocate data structures.";
            goto bad;
        }

        sc->io_pool = mempool_create_slab_pool(MIN_IOS, _io_pool);
        if (!sc->io_pool) {
            ti->error = "Cannot allocate mempool.";
            goto bad;
        }

        sc->page_pool = mempool_create_page_pool(MIN_POOL_PAGES, 0);
        if (!sc->page_pool) {
            ti->error = "Cannot allocate page mempool.";
            goto bad;
        }

        sc->bs = bioset_create(MIN_IOS, 0);
        if (!sc->bs) {
            ti->error = "Cannot allocate bioset.";
            goto bad;
        }
        // Note that the flag WQ_NON_REENTRANT no longer exists as all workqueues are now non-reentrant 
        //- any work item is guaranteed to be executed by at most one worker system-wide at any given time.
        //sc->queue = alloc_workqueue("sadcd", WQ_NON_REENTRANT | WQ_MEM_RECLAIM, 1);
        sc->queue = alloc_workqueue("sadcd", WQ_MEM_RECLAIM, 1);
        if (!sc->queue) {
            ti->error = "Cannot allocate work queue.";
            goto bad;
        }

        ret = -EINVAL;
        if (dm_get_device(ti, argv[0], dm_table_get_mode(ti->table), &sc->dev)) {
                ti->error = "dm-sadc: Device lookup failed.";
                return -1;
        }

        mutex_init(&sc->lock);
        init_completion(&sc->io_completion);

        /* TODO: Reconsider proper values for these. */
        ti->num_flush_bios = 1;
        ti->num_discard_bios = 1;
        ti->num_write_same_bios = 1;
        do_gettimeofday(&ct);
        last_io_begin_time = ct.tv_sec*1000+ct.tv_usec/1000;
        return 0;

bad:
        sadc_dtr(ti);
        return ret;
}

static int32_t sadc_map(struct dm_target *ti, struct bio *bio)
{
        struct sadc_ctx *sc = ti->private;
        struct io *io;

        // if (unlikely(bio->bi_opf & (REQ_OP_FLUSH | REQ_OP_DISCARD))) {
        //         WARN_ON(bio_sectors(bio));
        //         bio->bi_bdev = sc->dev->bdev;
        //         return DM_MAPIO_REMAPPED;
        // }
        
    switch (bio_op(bio)) {
        case REQ_OP_DISCARD:
        case REQ_OP_FLUSH:	
            //		case REQ_OP_SECURE_ERASE:
            //		case REQ_OP_WRITE_ZEROES:
            printk(KERN_INFO "Discard or Flush: %d \n", bio_op(bio));
            WARN_ON(bio_sectors(bio));
            bio->bi_bdev = sc->dev->bdev;
            return DM_MAPIO_REMAPPED;
        default:
            break;
    }

        io = alloc_io(sc, bio);
        if (unlikely(!io))
                return -EIO;

        queue_io(io);

        return DM_MAPIO_SUBMITTED;
}

static void sadc_status(struct dm_target *ti, status_type_t type,
                        unsigned status_flags, char *result, unsigned maxlen)
{
        struct sadc_ctx *sc = (struct sadc_ctx *) ti->private;

        switch (type) {
        case STATUSTYPE_INFO:
                result[0] = '\0';
                break;

        /* TODO: get string representation of device name.*/
        case STATUSTYPE_TABLE:
                snprintf(result, maxlen, "%s track size: %d, band size in tracks: %d, cache percent: %d%%",
                         sc->dev->name,
                         sc->track_size,
                         sc->band_size_tracks,
                         sc->cache_percent);
                break;
        }
}

// static int32_t reset_disk(struct sadc_ctx *sc)
// {
//         int32_t i;

//         DMINFO("Resetting disk...");

//         if (!mutex_trylock(&sc->lock)) {
//                 DMINFO("Cannot reset -- GC in progres...");
//                 return -EIO;
//         }

//         for (i = 0; i < sc->nr_cache_bands; ++i)
//                 reset_cache_band(sc, &sc->cache_bands[i]);

//         memset(sc->pba_map, -1, sizeof(pba_t) * sc->nr_usable_pbas);

//         mutex_unlock(&sc->lock);

//         return 0;
// }

// static int32_t sadc_ioctl(struct file *fp, unsigned int32_t num, unsigned long arg)
// {
//         // struct sadc_ctx *sc = ti->private;

//         // if (cmd == RESET_DISK)
//         //         return reset_disk(sc);
//         // return -EINVAL;
//         printk(KERN_INFO "ioctl %d\n", num);
// 	switch (num) {
// 		case 1:
// 			printk(KERN_INFO "ioctl 1\n");
// 			break;
// 		case 2:
// 			printk(KERN_INFO "ioctl 2\n");
// 			break;
// 		case 3:
// 			printk(KERN_INFO "ioctl 3\n");
// 			break;
// 		default:
// 			break;
// 	}

// 	return 0;
// }

static int32_t sadc_iterate_devices(struct dm_target *ti,
                                iterate_devices_callout_fn fn, void *data)
{
        struct sadc_ctx *sc = ti->private;

        return fn(ti, sc->dev, 0, ti->len, data);
}

// 注册为该类型的target type
static struct target_type sadc_target = {
        .name            = "sadc",      //定义模块的名字，下面定义各个函数
        .version         = {1, 0, 0},
        .module          = THIS_MODULE,
        .ctr             = sadc_ctr,    //target加载函数
        .dtr             = sadc_dtr,
        .map             = sadc_map,
        .status          = sadc_status,
        .prepare_ioctl   = 0,
        .iterate_devices = sadc_iterate_devices, // 迭代遍历源设备
};
//定义模块的初始化函数，用于注册target
static int32_t __init sadc_init(void)
{
        int32_t r;

        _io_pool = KMEM_CACHE(io, 0);
        if (!_io_pool)
                return -ENOMEM;

        r = dm_register_target(&sadc_target); //引用上面的静态结构体
        if (r < 0) {
                DMERR("register failed %d", r);
                kmem_cache_destroy(_io_pool);
        }

        return r;
}



static void __exit sadc_exit(void)
{
        dm_unregister_target(&sadc_target);
}

module_init(sadc_init);
module_exit(sadc_exit);

MODULE_DESCRIPTION(DM_NAME " set-associative disk cache STL emulator target");
MODULE_LICENSE("GPL");
