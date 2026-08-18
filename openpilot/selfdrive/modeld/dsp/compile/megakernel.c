// megakernel.c -- the FIXED fused DSP backbone interpreter, run as ONE tinygrad Tensor.custom_kernel.
// Model-independent: everything per-model arrives as DATA (the op list + weight blob codegen.py builds) or as
// -D defines (MegakernelLayout, in dsp/megakernel.py). The op-record contract below -- INTS_PER_OP and the
// OP_* opcodes -- is likewise -D'd in from codegen.py, so this file and the encoder cannot drift apart.
// Compile: build.compile_megakernel -- clang --target=hexagon -mcpu=hexagonv65 -mhvx=v65 -mhvx-length=128b -O2
//   -nostdlib -ffreestanding + the vendored nnlib .S kernels (see build.py's link list).


#include <hexagon_types.h>
#include <hvx_hexagon_protos.h>
#include "HAP_power.h"
typedef union { struct { void *pv; unsigned int len; } buf; struct { int fd; unsigned int offset; } dma; } remote_arg;
void* HAP_mmap(void *addr, int len, int prot, int flags, int fd, long offset);
int HAP_munmap(void *addr, int len);
unsigned long long HAP_perf_get_time_us(void);
// VTCM: 256KB of on-chip SRAM. MEASURED (membw_probe.c): same bandwidth as L2 (~76 GB/s) and WORSE latency (23 vs
// 16 cycles), so it is never "faster memory" -- its only value is CAPACITY THAT CANNOT BE EVICTED. That is exactly
// what the conv circular buffer wants: repstream writes it and gvconv re-reads it every output row, ~100KB for a
// distill 3x3, and it currently sits in the arena competing for the ~512KB of usable L2 against the weight group
// (320KB) and the activation stream. Phase 1 measured that the weight set IS evicted between rows (not re-fetching
// it costs +9.2%), so freeing 100KB of L2 is the direct attack on that eviction.
// ★★★ PHASE 3 RESULT (2026-07-25): DEAD, and decisively. Putting the conv circ buffer in VTCM made distill
// 9.88 -> 20.57 ms, +108%, 0 of 4 paired runs won. CONTROL RUN (VTCM_KB=1, so the block is requested and released
// but never large enough to be used) cost only +1.96%, which rules out the HAP request/release as the cause: the
// +10.7ms is the VTCM ACCESSES themselves. A ~100KB buffer that repstream writes and gvconv re-reads every row is
// simply not viable there -- VTCM is not backed by L1/L2 for these patterns, so every narrow access pays full
// VTCM latency with no line reuse, which dwarfs the 23-vs-16-cycle figure the microbenchmark reports.
// ⇒ This also kills the weight-group-in-VTCM idea: gvconv reads weights the same way, every cycle, so it can only
// be worse. VTCM has now failed three independent ways (equal bandwidth, worse latency, unusable as an operand
// buffer) plus a 2% cost just to ask for it. Do not propose VTCM again without NEW hardware information.
extern void to_d32_asm(const unsigned char* in, int in_width, unsigned char* d32, int next_width_d32, int in_height, int in_depth);
extern void from_d32_asm(const unsigned char* d32, int next_width_d32, unsigned char* out, int in_width, int in_height, int in_depth);
// ---- V65 circular-datapath conv (nnlib gvconv2dbbb_circ_d32_v65 + repstream2). SIGNED weights (vrmpy .b, no per-
// pixel suma -> the zp correction is folded into biasbuf via gemsumb), 8 out-px/iter, small pre-shuffled circular
// activation buffer. Makes the 1x1s COMPUTE-bound. weights = signed int8 in the same tile layout as V60.
extern void gvconv2dbbb_circ_d32_v65_asm(const unsigned char* input, const signed char* weights, unsigned char* output,
  int in_width_pad, int next_out_width_row, int out_width, int stride_w_h, int indepth, int filt_width, int filt_height,
  int num_out_lines, const int* ptr_wsum, int* ptr_max, const unsigned int* recip_level, int next_out_width,
  unsigned char* circ_buffer, int zshift, int in_offset, const unsigned char* store_ctrl);
extern void gvconv2dbbb_circ_d64_v65_asm(const unsigned char* input, const signed char* weights, unsigned char* output,
  int in_width_pad, int next_out_width_row, int out_width, int stride_w_h, int indepth, int filt_width, int filt_height,
  int num_out_lines, const int* ptr_wsum, int* ptr_max, const unsigned int* recip_level, int next_out_width,
  unsigned char* circ_buffer, int zshift, int in_offset, const unsigned char* store_ctrl);
extern void repstream2_asm(const unsigned char* input, unsigned char* output, int width, int depth, int fill_height,
  int rpad_lpad, int stride_w, const unsigned char* circ_base, int buf_height, int in_offset, int num_accs);
// per-position store masks for the circ kernel's tail (partial d32 pixel groups); copied verbatim from op_supernode_new.c.
// nnlib copy3to4 vdelta control (op_supernode_new.c:4879): reshuffles 96 input bytes (32 px x 3ch) -> 128 out bytes
// (32 px x 4ch), 4th byte then muxed to the pad value. Used by the PIL stem's HVX copy3to4 (stem_pil_fill).
static const unsigned char copy3to4_cntrl[128] __attribute__((aligned(128))) = {
  0x00,0x00,0x00,0x00,0x01,0x03,0x01,0x06,0x02,0x02,0x06,0x04,0x03,0x05,0x0D,0x0C,
  0x04,0x04,0x04,0x00,0x0D,0x0F,0x09,0x0A,0x06,0x02,0x0A,0x08,0x1B,0x19,0x19,0x18,
  0x08,0x08,0x08,0x08,0x09,0x0B,0x01,0x06,0x1A,0x1A,0x1E,0x1C,0x13,0x15,0x15,0x14,
  0x0C,0x0C,0x04,0x00,0x15,0x17,0x11,0x12,0x36,0x32,0x32,0x30,0x33,0x31,0x31,0x30,
  0x10,0x10,0x10,0x10,0x11,0x13,0x11,0x16,0x12,0x12,0x16,0x14,0x03,0x05,0x0D,0x0C,
  0x34,0x34,0x34,0x30,0x3D,0x3F,0x39,0x3A,0x26,0x22,0x2A,0x28,0x2B,0x29,0x29,0x28,
  0x18,0x18,0x18,0x18,0x09,0x0B,0x01,0x06,0x2A,0x2A,0x2E,0x2C,0x23,0x25,0x25,0x24,
  0x6C,0x6C,0x64,0x60,0x65,0x67,0x61,0x62,0x66,0x62,0x62,0x60,0x63,0x61,0x61,0x60,
};
static const unsigned char store_cntrl[384] __attribute__((aligned(128))) = {
  0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
  0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
  0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,
  0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,
  0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,
  0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,0x7,
  0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,
  0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,0xf,
  0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
  0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
  0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,
  0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,0x3,
  0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,
  0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,0x6,
  0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,
  0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,0xc,
  0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,
  0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,0x8,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
};

static inline void l2f(const void* p, unsigned width, unsigned height){    // prefetch height x width bytes, stride=width
  unsigned long long ctl=(((unsigned long long)((1u<<16)|width))<<32)|((width<<16)|height);
  asm volatile(" l2fetch(%0,%1) "::"r"(p),"r"(ctl));
}
// ---- PHASE TIMERS (P0.1 follow-up). Per-op totals live at prof[16+i]; slots 240+ are free for both models
// (both models' `out` holds >=256 u64: distill 512 floats, MNv2 1000) and hold PHASE aggregates for the run. They exist because the per-op
// table showed a large SIZE-INDEPENDENT term -- MNv2's depthwise fit `us = 60.9 + 1.16*floor`, i.e. ~1.0ms of
// the model is a constant per-dw cost that no prefetch or residency change can touch. These say WHICH phase.
// The slots, in one block. PER-OP times live at prof[16+i] and answer "WHICH OP is slow"; these answer "which
// PHASE WITHIN an op class", which is a different question and the one that found the SE GAP (0.404ms, task #36)
// and dw.repad (0.594ms, task #71). Both are worth having; only the ergonomics of these needed fixing.
//   SE_RQ    the per-channel requant loop        DW_TILE    the whole tile loop (asm + from_d32), calling thread
//   SE_FC2   fc2 + sigmoid index + LUT gather    DW_UNPACK  from_d32 inside the worker -- SUMMED ACROSS THREADS,
//   GV/REP   inside the gvconv / repstream asm              so it reads HIGHER than wall when threaded
//   ROW_US   the whole per-row body (residual)   CV_REPAD   conv_repad_d32 only, split out of CV_PACK
enum { PH_CV_PACK=216, PH_CV_COMPUTE, PH_CV_UNPACK, PH_CV_REPAD=229,
       PH_GV_US=236, PH_REP_US, PH_ROW_US,
       PH_SE_GAP=240, PH_SE_RQ, PH_SE_FC1, PH_SE_FC2, PH_DW_REPAD, PH_DW_TILE, PH_DW_UNPACK,
       PH_PF_ISSUE, PH_PF_CLOB, PH_PF_LIVE };
// ---- P0.2: is an l2fetch actually CANCELLED by the next one? V65 manual 5.10.6 says issuing l2fetch while
// USR.PFA (bit 31) is set halts and overwrites the prefetch in flight. Everything in this file assumes requests
// accumulate, and we issue very large ones (a 512KB input span, a per-row weight set up to 320KB, a 64KB cross-op
// blast) -- all far over the manual's <8KiB advice, so each is in flight a long time and is a big target.
// PF_CLOB/PF_ISSUE is the clobber RATE; PF_LIVE counts how often a fetch is still running when its consumer starts.
static inline int pfa(void){ unsigned u; asm volatile("%0 = usr":"=r"(u)); return (int)((u>>31)&1); }
// l2f + the two counters. `prof` may be NULL (non-PROF builds and every uninstrumented caller).
#define L2FP(pf, p, w, h) do{ if(pf){ (pf)[PH_PF_ISSUE]++; if(pfa()) (pf)[PH_PF_CLOB]++; } l2f((p),(w),(h)); }while(0)
// PER-SITE clobber attribution (slots 200+, which sit between the per-op times and the phase timers). The aggregate
// 37% says clobbering happens; it does not say WHICH stream overwrites which, and PF-2 was built on a guess about
// that which turned out to be wrong. Counting at each site is free and answers it directly -- much better than
// ablating a site, which changes behaviour AND the binary layout.
// ★ conv_op's per-OP phases. They USED to live at prof[5]/[6]/[7]/[8..11] and SILENTLY COLLIDED with
// prof[op[0]], the per-opcode totals: OP_DWCONV=5, OP_ADD=6, OP_INCONV=8, OP_PACK=11. Any conclusion drawn from
// those slots was the sum of two unrelated things.
// ★★★ PHASE 1 RESULT (2026-07-25): the clobbering is REAL -- 36.8% of distill's l2fetches and 35.5% of MNv2's are
// issued while USR.PFA is set, i.e. they overwrite a prefetch in flight, exactly as V65 manual 5.10.6 describes.
// It is also NOT WORTH FIXING, because every remedy that works by refreshing a stream LESS OFTEN costs far more
// than the clobbering does. All paired, alternating, on-DSP medians, 0 of N pairs won:
//     PF_ALT=1     (input on even rows, weights on odd)   distill +12.3%
//     PF_WT_ROW=0  (drop the per-row weight re-fetch)     distill  +9.2%,  MNv2 +2.7%
//     PF_GATE=1    (skip an issue while one is in flight) distill  +7.1%
// ⇒ A CLOBBERED l2fetch IS NOT A WASTED l2fetch. It runs until it is overwritten and delivers the lines it got
// to, so partial delivery every row beats full delivery every other row. Do not "fix" the 36% again.
// The knobs themselves are DELETED -- the numbers above are the result, and a default-off knob for a measured
// loss is just a trap. The two remedies that would keep BOTH streams at full rate were also tried and both
// failed: a prefetch-only thread wedges the DSP (see the pool below) and VTCM is 2x slower (see the top).
// ★★★ PHASE TIMERS MUST NOT USE HAP_perf_get_time_us. It is a CALL, and at this granularity it costs more than
// the thing being timed: the T1a border zap read 0.390ms across 16 depthwises, but ABLATING THE WHOLE LOOP saved
// only 0.011ms (paired, 4/4). ~97% of that reading was the timer. `utimer` is a user-readable free-running
// 19.2MHz register -- one instruction, no call, no clobber. Coarse (52ns) but these are run-total aggregates.
// The PER-OP timers at prof[16+i] are NOT affected: two calls around an 80us op is noise, and their sum was
// checked against the whole-model time. Trust per-op numbers; treat any phase split as needing an ablation.
static inline unsigned long long utimer(void){ unsigned long long t; asm volatile("%0 = utimer":"=r"(t)); return t; }
#define PH_NOW (prof ? utimer() : 0)
// PHASE(slot){ ... } times a BLOCK. A for-scope, so the timer variable is declared in it, the body runs exactly
// once, and the accumulate happens in the increment clause -- which makes a timed region a visible block instead
// of three lines of hand-rolled temporary (`unsigned long long _t=PH_NOW; ...; PH_T(slot,_t);`) plus a brace
// block existing only to scope that temporary. Nesting works (the inner _t0 shadows the outer).
// The accumulate must be an EXPRESSION, not the `do{...}while(0)` statement the old PH_T macro was -- a for
// increment clause takes an expression, so the statement form fails to compile in every nested use.
// CAVEAT: `break` or `continue` in the body would skip the accumulate; do not time a region containing either.
#define PHASE(slot) for(unsigned long long _t0=PH_NOW, _once=1; _once; _once=0, (void)(prof && (prof[slot]+=utimer()-_t0)))
// ★★★ THE SCALAR NHWC->d32 pack() IS GONE (2026-07-26), and with it the last non-vectorized layout path.
// It was ~40 lines of hand-tuned scalar copy (blk32_t existed purely to promise clang 8-byte alignment so a
// 32-byte pixel copy became 4 memd instead of 32 memb), and by the end NEITHER GATE MODEL CALLED IT: distill is
// 100% d32-resident, and MobileNetV2's one caller -- the first depthwise, packing the stem's output -- stopped
// when _conv_std learned that at Cout==32 the stem's NHWC output ALREADY IS d32 (see lower.py).
//
// What that leaves is a cleaner invariant than "we have a fallback": **NHWC exists only at the graph's entry, and
// there are exactly two entry kernels** -- OP_PACK (to_d32_asm) for a mult-32 seed, and the PIL stem for a
// <=4-channel one. Everything downstream is d32 by construction. A model that needs neither (a non-mult-32 seed
// feeding an ordinary conv) now fails the BUILD with a message naming the op, instead of silently running a
// scalar path at 2.33 ns/B against to_d32_asm's ~0.2. Same fast-or-fail trade as the deleted scalar unpacks and
// se_gate fallbacks: refuse loudly rather than ship a silent 10x.
// Unaligned HVX/scalar load typedefs. Hexagon FAULTS on a misaligned vector load, so anything whose address is only 32-byte aligned
// (d32 chunk rows, blob operands) must load through an aligned(1) typedef and let clang emit vmemu.
typedef HVX_Vector HVX_UVec __attribute__((aligned(1)));
typedef unsigned uint_u __attribute__((aligned(1)));   // scalar unaligned word load (clang emits unaligned memw, no fault)
// d32 block copy/zero via HVX unaligned vectors (vmemu). The dw REPAD used align-1 __builtin_memcpy/memset -> byte
// stores at ~0.66 GB/s (op4 s2 repad = 1.9ms). A blk32_t struct-copy loop is WORSE (clang lowers each 32B assign to a
// memcpy CALL). Explicit vmemu intrinsics can't be idiom-rewritten to calls -> fast, and fix BOTH s1 and s2 (no
// alignment constraint on padL). HVX is held on this (main) thread. Sizes here are whole 32B pixels; <128B tail = bytes.
static inline void d32_copy(unsigned char* d, const unsigned char* s, long nbytes){
  HVX_UVec* dp=(HVX_UVec*)d; const HVX_UVec* sp=(const HVX_UVec*)s; long nv=nbytes>>7;
  for(long i=0;i<nv;i++) dp[i]=sp[i];
  for(long b=nv<<7;b<nbytes;b++) d[b]=s[b]; }
// ALIGNED copy: both pointers 128-aligned and the length a whole number of vectors. Separate from d32_copy
// because that one takes HVX_UVec* for both operands and so emits vmemu NO MATTER WHAT THE ADDRESS IS -- the
// alignment has to be in the TYPE. Enabled by the padL=4 input geometry (see conv_geom in lower.py).
static inline void d32_copy_a(unsigned char* d, const unsigned char* s, long nbytes){
  HVX_Vector* dp=(HVX_Vector*)d; const HVX_Vector* sp=(const HVX_Vector*)s; long nv=nbytes>>7;
  for(long i=0;i<nv;i++) dp[i]=sp[i]; }
// ★ AN ALIGNED FAST PATH HERE IS A LOSS, unlike in d32_copy. Adding a runtime alignment test that picks
// HVX_Vector* stores made the dw probes +2.6..+4.7% and MNv2's dw.repad 0.594 -> 0.625 ms. That fits the
// earlier ablation: this zeroing's value is partly that it WARMS the bordered buffer into L2 just before the
// depthwise streams it, so making it finish sooner removes some of the warming. Speed is not the objective here.
static inline void d32_zero(unsigned char* d, long nbytes){
  HVX_UVec* dp=(HVX_UVec*)d; HVX_Vector z=Q6_V_vzero(); long nv=nbytes>>7;
  for(long i=0;i<nv;i++) dp[i]=z;
  for(long b=nv<<7;b<nbytes;b++) d[b]=0; }
static inline void d32_fill(unsigned char* d, int val, long nbytes){   // fill with a byte (activation zp) for d32 borders
  unsigned r=((unsigned)val&0xff)*0x01010101u; HVX_UVec* dp=(HVX_UVec*)d; HVX_Vector v=Q6_V_vsplat_R(r); long nv=nbytes>>7;
  for(long i=0;i<nv;i++) dp[i]=v;
  for(long b=nv<<7;b<nbytes;b++) d[b]=(unsigned char)val; }
// non-temporal aligned HVX store: writes to DDR/L2 WITHOUT a write-allocate (no L2 line pulled in). Use for output
// that is never re-read by the producing op -> avoids evicting still-needed input/weights from L2 (nnlib uses :nt for
// all conv output). p MUST be 128-aligned.
static inline void vstnt(void* p, HVX_Vector v){ asm volatile("vmem(%0+#0):nt = %1"::"r"(p),"v"(v):"memory"); }
// ★ CARVE A SUB-BUFFER OUT OF THE SHARED SCRATCH. The FastRPC DSP thread stack is 16 KB and interp() inlines every
// kernel into one frame, so a per-model working array MUST NOT live on the stack -- and a fixed-size one sized for
// today's model is worse still: `gqtab[2048]` (needs C*4 shorts) and `fc1u[128]` (needs Csq floats) each ran past
// their end on a wide model, the first silently corrupting the frame and the second faulting the PD. Every such
// array now comes from here. Each take is 128-ALIGNED (HVX aligned vector loads); the op's TOTAL is declared
// host-side by lower.py's `scr(...)`, which lists the same sizes in the same order, and build.scratch_layout takes
// one max over every op's declaration. Adding a take here means adding its size to that one call -- and nowhere else.
#define SCR_TAKE(p, bytes) ({ void* _r__ = (void*)(p); (p) += (((bytes) + 127) & ~127); _r__; })
typedef int HVX_VecW __attribute__((vector_size(128)));   // 32 int32 lanes (unambiguous vs hexagon_types' HVX_Vector)
typedef int HVX_VecWU __attribute__((vector_size(128), aligned(1)));   // same, unaligned load (vmemu) for blob operands
extern int qurt_hvx_lock(int mode);     // a spawned thread MUST lock an HVX unit to run HVX in parallel (QURT_HVX_MODE_128B=1)
extern int qurt_hvx_unlock(void);       // V65/685 = 2 HVX units -> main thread + 1 locked worker = 2-way
// QURT native threads (real HW threads) -- tinygrad's DSP MULTICORE uses these (NOT pthread, which time-slices one HW
// thread -> no memory parallelism). Real HW threads give independent outstanding memory requests -> ~N x memory BW.
// QuRT priority: 0 = HIGHEST, 255 = LOWEST. The pool worker ran at 255, so the main FastRPC thread starved it and
// pool_run2 measured "== single" while pthread (which INHERITS the caller's priority) measured 2x. That single number
// is why the compute-bound V65 convs were routed to per-op pthread_create (~0.1ms spawn each) instead of the pool.
// ★ PF-2 MEASURED DEAD 2026-07-25. Both threads have their OWN prefetch engine (pfcal.py: two threads fetching
// concurrently each make progress, aggregate 13.8 vs 11.6 GB/s for one), and thread 1 currently issues no weight
// fetch at all, so splitting the shared weight re-fetch across the two engines by row parity looked free. It is
// not a win and, more importantly, it DOES NOT DO WHAT IT WAS BUILT TO DO: the clobber rate did not move (distill
// 37.1 -> 37.0%, MNv2 36.0 -> 36.2%), and per-op attribution put conv ops at +0.24% (distill) / +2.42% (MNv2)
// while the whole-model A/B read -0.62% -- i.e. the apparent win was layout, and the mechanism never fired.
// Per-site clobber counters (since deleted) then explained why -- see the row-weight fetch below.
#ifndef V65_ONE_WT_PF
#define V65_ONE_WT_PF 1
#endif
#ifndef POOL_PRIO
#define POOL_PRIO 100    // swept 32/64/100/128/160 on distill: 100 best, flat 64-128, 255 = starved
#endif
typedef struct { char name[16]; unsigned char tcb_partition, affinity; unsigned short priority; unsigned char asid,
  bus_priority; unsigned short timetest_id; unsigned int stack_size; void* stack_addr; char padding[96]; } qurt_thread_attr_t;
extern int qurt_thread_create(unsigned long*, qurt_thread_attr_t*, void(*)(void*), void*);
extern int qurt_thread_join(unsigned long, int*);
extern void qurt_thread_exit(int);
extern void* malloc(unsigned int);
extern void free(void*);
#ifndef CONV_NT
#define CONV_NT 2      // conv row-split fan-out (V65/685 = 2 HVX units). qurt_thread+qurt_hvx_lock = real HW
#endif                 // parallelism: 1.58x on compute-bound convs (dense 3x3). A/B via -DCONV_NT=N.
// ---- ★ THE CONV THREADING GATES ARE GONE (2026-07-26), because every one of them had been swept to a
// value that made it ALWAYS TRUE. POOL_SPATIAL, POOL_DENSITY and POOL_WORK all sat at 1, so the "is this conv
// compute-bound enough to thread" test they existed to express -- three terms, ~14 lines of comment, one
// autotune knob each -- had reduced to `nt>1`. They were written when the pool was started only for graphs with
// depthwise; once V65_POOL_CONV (also now unconditional) started it for any conv graph, the answer became "yes,
// always" and the sweeps kept confirming it. PTHREAD_WORK went with them: it gated the non-pooled path, which
// is now unreachable (see conv_op). What remains is the one decision that survived: 2 threads when there is
// more than one output row, 1 otherwise.
// ★★★ PHASE 5 / E5.2 ATTEMPTED AND WITHDRAWN (2026-07-25): a PREFETCH-ONLY hardware thread. The idea was sound on
// paper -- V65 has 4 hardware contexts but only 2 HVX units, and `l2fetch` is a SCALAR instruction needing no HVX
// context, so a third thread could own the weight stream and give it its OWN prefetch engine (USR.PFA is per
// thread). That is the only remedy Phase 1 could not test, since every other fix had to reduce a stream's refresh
// rate and all of those lost. Implemented as a qurt thread spinning on a published {ptr,bytes,seq} slot.
// IT WEDGED THE DSP -- rc=39, and unlike the usual transient rc=39 it did NOT clear on its own in 12 minutes,
// which is the signature of leaked SPINNING QuRT threads inside the CDSP PD rather than a stale FastRPC context.
// Retrying with a 64KB stack (vs 8KB) and a lower priority did not help. THE PREMISE IS WRONG IN PRACTICE: the 4
// contexts are not free -- FastRPC's own invoke thread takes one, the pool worker a second, and the non-pooled
// convs still `pthread_create` a third, so a permanently-spinning fourth leaves the DSP nothing. Note also that
// POOL_SPIN already measured that an extra spinning thread hurts short ops. Do not retry without first making the
// prefetcher event-driven rather than spinning, AND proving the thread is joined on every exit path.
// ---- PERSISTENT thread pool: ONE qurt worker per interp() call (holds HVX unit 1), barrier-synced per op. Avoids the
// per-op spawn (~0.1ms x ~17 dws = net loss). Uses QURT_THREAD + qurt_barrier (NOT pthread: pthread+qurt_barrier wedges
// FastRPC = rc=39; tinygrad's working MULTICORE uses qurt_thread+qurt_barrier). NO STATICS (loader segment cap) -> the
// pool_t is on interp()'s STACK, passed by pointer. Statics-free, so no new LOAD segment.
typedef struct { char pad[64]; } qurt_barrier_t;
extern int qurt_barrier_init(qurt_barrier_t*, unsigned int);
extern int qurt_barrier_destroy(qurt_barrier_t*);
extern int qurt_barrier_wait(qurt_barrier_t*);
typedef struct { qurt_barrier_t bstart, bdone; void*(*volatile fn)(void*); void* volatile arg; volatile int stop, up;
  unsigned long th; void* stack; } pool_t;
// ★ SPIN vs BARRIER, SETTLED 2026-07-25, spin DELETED 2026-07-26. `qurt_barrier_wait` BLOCKS, so the worker is
// rescheduled onto a hardware thread every op -- a fixed wake latency, visible as a 2-way speedup that GROWS with
// work per op (1.16x at Ho=8, 1.24x at Ho=16, 1.33x at Ho=32). A spinning worker removes that, and it does help
// the real models (distill 10.02->9.92, MNv2 5.63->5.52, ~1-2%), but it REGRESSES the short ops (conv2x2s2 +6%;
// dw_k3/k5_s1 lose their gains) because the spinner burns a HW thread they need. It was kept as a default-off
// #if for A/B, which is the trap this codebase keeps retiring: a second implementation of the pool's whole
// synchronisation, compiled never, maintained forever. The numbers above are the result; the code was not.
static void pool_worker(void* a){                 // qurt thread entry (void return)
  pool_t* p=a;
  qurt_hvx_lock(1);                               // grab HVX unit 1 for the pool's lifetime
  for(;;){
    qurt_barrier_wait(&p->bstart);                // block (no spin) until the main hands out work
    if(p->stop) break;
    p->fn(p->arg);
    qurt_barrier_wait(&p->bdone);
  }
  qurt_hvx_unlock();
  qurt_thread_exit(0);
}
static void pool_start(pool_t* p){
  p->stop=0; p->up=0; p->stack=0;
  qurt_barrier_init(&p->bstart,2); qurt_barrier_init(&p->bdone,2);
  qurt_thread_attr_t attr; __builtin_memset(&attr,0,sizeof(attr));
  attr.name[0]='p'; attr.priority=POOL_PRIO; attr.stack_size=(64<<10); attr.stack_addr=malloc(attr.stack_size); p->stack=attr.stack_addr;
  if(attr.stack_addr && qurt_thread_create(&p->th,&attr,pool_worker,p)==0) p->up=1;
  else { qurt_barrier_destroy(&p->bstart); qurt_barrier_destroy(&p->bdone); if(p->stack) free(p->stack); }
}
static void pool_stop(pool_t* p){
  if(!p->up) return;
  p->stop=1;
  qurt_barrier_wait(&p->bstart);                  // release worker -> sees stop -> exits (no bdone this round)
  int st; qurt_thread_join(p->th,&st);
  qurt_barrier_destroy(&p->bstart); qurt_barrier_destroy(&p->bdone); if(p->stack) free(p->stack); p->up=0;
}
static void pool_run2(pool_t* p, void*(*fn)(void*), void* arg0, void* arg1){
  if(!p || !p->up){ fn(arg0); fn(arg1); return; }
  p->fn=fn; p->arg=arg1;
  qurt_barrier_wait(&p->bstart);                  // release worker onto arg1
  fn(arg0);                                       // main runs arg0 in parallel
  qurt_barrier_wait(&p->bdone);                   // rendezvous
}
// V65 circular-datapath worker: produces output rows [oy0,oy1) using its OWN circ-buffer slice + HVX unit. Per row:
// repstream the next input row, then conv all out chunks (func32 for the odd chunk, func64 for the rest -> 64 out
// channels per activation load = weight reuse). buf_height=1 for 1x1 s1 (cbuf_row stays 0). Weights are read-only ->
// shared across threads; the circ buffer + minmax are per-thread (on the worker's own slice/stack).
typedef struct { unsigned char *inp,*outp,*circ,*weights; int *biasbuf,*recip;
  int Win,Cigp,in_width_pad,buf_height,in_next_row,stride_wh,rpad_lpad,sw,sh,kh,kw,Wo,Wop,out_next_row,out_chunks,
      w_gcstride,zshift,xzp,oy0,oy1,lock,ochunk_w,pf_wt; long buf_width; unsigned long long* prof; } v65work_t;   // ochunk_w=per-chunk out width (T1a bordered=Wp)
static void* v65_tile_worker(void* arg){
  v65work_t* w=arg; unsigned long long* prof=w->prof;
  if(w->lock) qurt_hvx_lock(1);
  int mm[64] __attribute__((aligned(128)));
  for(int i=0;i<32;i++){ mm[i]=-0x7fffffff; mm[32+i]=0x7fffffff; }
  int n_in_rows = w->kh<w->sh ? w->kh : w->sh;
  long inr=w->in_next_row;
  // A thread producing output rows [oy0,oy1) needs input rows starting at padded row oy0*sh (the sliding window for
  // output oy reads rows [oy*sh, oy*sh+kh)). oy0*sh, NOT oy0 -- for stride>1 the two differ, and the initial fill /
  // upfront prefetch below both index off this base. (Single-thread oy0=0 and s1 sh=1 are unaffected; the bug only bit
  // threaded stride-2 convs -- thread 1's first rows read stale rows oy0.. instead of oy0*sh.. -> wrong for 2 rows.)
  long in_row0 = (long)w->oy0*w->sh;
  // Prefetch the WHOLE input slice [oy0,oy1) upfront (capped) so the per-row repstream reads hit L2 instead of
  // stalling on DDR latency one row at a time -- these shallow convs are read-latency-bound (no MLP), not compute-bound.
  // ★★★ 2026-07-25: this cap was 512KB and is now 64KB. RETAINED ON MECHANISM, NOT ON A MEASURED WIN -- the
  // -0.91%/-1.11% first claimed for it was an ARTIFACT of a biased A/B harness (see floors.ab: the arm loaded
  // second measures ~1% faster because it lands at a different ION placement; a null control of two equivalent
  // binaries "won" 5/5). Order-cancelled it reads distill -0.55%, MNv2 +0.10%, against a null control that reads
  // +0.6% -- i.e. INDISTINGUISHABLE FROM ZERO. Do not quote a number for this change.
  // The mechanism argument is what justifies it, and that IS measured (pfcal.py, and none of it is an A/B):
  //   * the engine delivers 13.3 GB/s with ~0.25us startup, so the prefetch DISTANCE a conv needs is under 2KB;
  //     a 512KB request buys no extra earliness, and at ~10us per row it cannot even be delivered within the row.
  //   * L2 retains ~384KB (warm re-read holds 75.7 GB/s to 384KB, collapses to 31.4 at 512KB), and a resident
  //     128KB block survives 256KB of streaming but is annihilated by 512KB (survives 0.03).
  //   * so 512KB input + 320KB weights asked for 832KB of a 384KB cache. 64KB+320KB fits. Smaller cannot be worse.
  // Shrinking the WEIGHT group to pay for it does lose by margins above the noise floor (128K/256K +3.90%,
  // 192K/192K +2.87%), so V65_WT_BUDGET=320K stands and only the input request was oversized.
#ifndef PF_IN_CAP
#define PF_IN_CAP (64*1024)
#endif
  { long span=(long)((w->oy1-w->oy0-1)*w->sh + w->kh)*inr; if(span>PF_IN_CAP) span=PF_IN_CAP;
    unsigned nb=(unsigned)(span/128); if(nb>65535) nb=65535; if(nb) L2FP(prof, w->inp+in_row0*inr, 128, nb); }
  // ★WEIGHT-TILING: V65 re-reads the FULL weight set (out_chunks*w_gcstride) every output row (Ho times). If that set
  // exceeds L2 it SPILLS -> every row re-reads it from DDR (measured: distill op31 256->512 3x3, 1.15MB wts, Ho=8 =
  // 9.2MB DDR re-read ~1.35ms). FIX: process the out-chunks in L2-FITTING GROUPS -- refill the circ (input) per group
  // and sweep the rows, so each group's weights are re-read from L2 not DDR. Input is small & stays L2-resident, so the
  // per-group input re-read is cheap. When the whole set fits (gsize==out_chunks) this is one pass == prior behavior.
  long w_gcs = w->w_gcstride;
// ★ THIS KNOB IS AT ITS OPTIMUM, RE-VERIFIED. The conv is weight-set-residency bound (see the block at the
// gvconv call for the evidence), so this budget is the knob that governs it. Re-swept after PF_IN_CAP dropped
// from 512K to 64K -- its 320K had been tuned when the total ask was 832KB, which swamped it. Still optimal:
//   conv ms:  320K 6.503 | 256K 7.152 | 192K 6.797 | 128K 6.693 | 96K 6.886
// Shrinking it trades weight residency against extra per-group input re-streams, and loses.
#ifndef V65_WT_BUDGET
#define V65_WT_BUDGET (320*1024)   // per-group weight budget. RE-CONFIRMED 2026-07-25 with a PAIRED A/B after the
                                   // threading fix, since the original sweep was single-shot on one op: 384K is
                                   // +1.1% and 224K +5.8% against this 320K baseline, so it is a real optimum and
                                   // the "usable L2 is really ~384K" hypothesis does not hold here.
                                   // Original: SWEPT on distill op31 (14.71ms @320K vs 14.85@400K,
#endif                             // 14.93@256K, 15.26@448K): weights(group) + input must CO-FIT L2 (~512K usable) --
                                   // 448K weights + 128K input = 560K > L2 regresses. Output streams out (not re-read).
  int gsize = w->out_chunks;
  if((long)w->out_chunks*w_gcs > V65_WT_BUDGET){ gsize = (int)(V65_WT_BUDGET / w_gcs); if(gsize<1) gsize=1; }
  for(int gc0=0; gc0<w->out_chunks; gc0+=gsize){
    int oc = (gc0+gsize <= w->out_chunks) ? gsize : (w->out_chunks - gc0);   // this group's out-chunk count
    unsigned char* gwt = w->weights + (long)gc0*w_gcs;
    int* gbias = w->biasbuf + gc0*32; int* grecip = w->recip + gc0*32;
    unsigned char* goutp = w->outp + (long)gc0*w->ochunk_w*32;   // group writes chunks [gc0,gc0+oc) at the full row stride
    long wtot=(long)oc*w_gcs; unsigned wnb=(unsigned)(wtot/128); if(wnb>65535) wnb=65535;
    if(wnb && w->pf_wt) L2FP(prof, gwt, 128, wnb);
    for(int i=0;i<32;i++){ mm[i]=-0x7fffffff; mm[32+i]=0x7fffffff; }   // reset minmax per group
    repstream2_asm(w->inp+in_row0*inr, w->circ, w->Win, w->Cigp, w->kh, w->rpad_lpad, w->sw, w->circ,
                   w->buf_height, w->xzp, 8);   // initial fill: rows [oy0*sh, oy0*sh+kh) for output row oy0
    int cbuf_row=0;
    for(int oy=w->oy0; oy<w->oy1; oy++){
      unsigned char* cin = w->circ + (long)cbuf_row*w->buf_width;
      unsigned char* orow = goutp + (long)oy*w->out_next_row;
      // The row(s) ENTERING the window for the next output row start at padded row oy*sh+kh (NOT oy+1 -- that only holds
      // for kh==1). For kh>1 the window slides by sh; the new bottom row is oy*sh+kh. (Fixes V65 for kh>1: was duplicating
      // a window row and dropping the real new row -> cos 0.99.) prefetch + repstream both read from this row.
      long newrow = (long)oy*w->sh + w->kh;
      // ABLATED 2026-07-25 on distill (both prefetches EARN their keep): dropping the input fetch costs +5.2%,
      // dropping the weight re-fetch +1.2%, and SWAPPING their order is neutral (+0.4%, inside noise).
      // ★ That last one REFUTES the l2fetch-cancellation theory -- these two are issued back to back with no work
      // between them, and if the second terminated the first, "weights only" would have measured the same as
      // baseline instead of 5.2% worse. Both fetches land. Do not "fix" the adjacency.
      // Capping the upfront input fetch (512K -> 256/128/64K) to co-fit L2 with the 320K weight group is ALSO a
      // NULL result: a paired alternating re-measure put 128K 7us SLOWER, after a single-shot run had said -0.8%.
      // Still untested: the manual's actual advice -- a ROLLING window of <8KiB fetches issued a few tiles ahead,
      // which is a different SHAPE, not a smaller bulk request.
      if(oy < w->oy1-1){ unsigned nb=(unsigned)((long)inr*n_in_rows)/128; if(nb) L2FP(prof, w->inp+newrow*inr, 128, nb); }
      // ★ ONE prefetcher for the SHARED weights: the 2-way split is over ROWS, so both threads sweep the SAME
      // weight set. L2 is shared, so a second l2fetch of the same lines only burns prefetch bandwidth. pf_wt is
      // set on thread 0 only.
      // ★★ THIS is where the 37% clobbering comes from (measured per site with counters since deleted): the weight fetch is
      // issued FOUR LINES after the input fetch with no work in between, and finds it still in flight 81.5% of the
      // time (MNv2 91.2%), while the input fetch itself -- issued after a whole row of compute -- finds the engine
      // busy only 12% (MNv2 5%). So the row-input prefetch on THREAD 0 is cancelled almost every row; thread 1
      // issues no weight fetch, so its input prefetch always survives, and that is what the +5.2% ablation of the
      // input fetch was really measuring. (The old comment here argued the +5.2% REFUTED cancellation. It does
      // not: a cancelled fetch still delivers what it got to, and thread 1's fetch is never cancelled at all.)
      // RE-MEASURED after PF_IN_CAP dropped the working set from 832K to 384K, on the theory that the group would
      // now stay resident and the repair would be unnecessary: it is still +8.50% (MNv2 +1.71%), 0/5. So the
      // evictor is the conv's own DEMAND stream -- rows of activations in and out, every row -- not the prefetch
      // requests. Consistent with pfcal.py's interference result (512KB of streaming annihilates a resident
      // block). The per-row weight re-fetch is not redundant and cannot be removed by shrinking other requests.
      if(wnb && w->pf_wt) L2FP(prof, gwt, 128, wnb);   // keep the re-read weight set streaming into L2 (overlaps the conv)
      if(prof && pfa()) prof[PH_PF_LIVE]++;   // still prefetching as the conv starts consuming
      // ★★★★★ WHAT BINDS THIS CONV, and the exclusion list. Read this before proposing anything here.
      // BINDING CONSTRAINT: WEIGHT-SET RESIDENCY IN L2. Sweeping the weight-region size in an otherwise identical
      // 2-thread loop (pfcal.py) gives 1.49 / 1.25 / 1.17 / 0.92 / 0.13 vrmpy-cycle at 128 / 256 / 384 / 512 /
      // 1024 KB. The real conv's 1.07 lands between the 384 and 512KB points -- exactly where a V65_WT_BUDGET
      // group plus activation traffic sits. The curve is STEEP (384->512 costs 21%), which is why the measured
      // ~384KB usable L2 governs everything here. V65_WT_BUDGET was re-swept afterwards and is still optimal.
      // ⇒ The only remaining lever on the conv is FEWER OR SMALLER WEIGHTS -- re-distilling, not a runtime change.
      //
      // THE STEADY-STATE STALL is a store-to-load handoff ACROSS A CACHE LEVEL: repstream2 writes the circular
      // buffer with HVX VECTOR stores (which land in L2) and this loop reads it back with 8 scalar memd per 9
      // packets (which want L1). Measured (pfcal.py S2, 8KB region so capacity is not the variable): a memd read
      // costs 111us after a memd read, 166us after an HVX vector store, 87us after a vector store + dcfetch.
      // The in-loop `dcfetch` is what makes this viable at all -- removing it costs +30.4% of gvconv.
      //
      // THE PREFETCH IS ALREADY RATE-MATCHED, DISTANCE-CORRECT AND COMPLETE, so there is nothing to add. `stride`
      // is `asl(stride_h_w,#5)` = 32*stride_w, a PIXEL stride: the four memd offsets (0, stride<<1, stride<<2,
      // stride3<<1) are 0/64/128/192 bytes = ONE CONTIGUOUS 256-BYTE WINDOW, four cache lines, which is what the
      // vendored comments mean by "load pt 0,4 / 1,5 / 2,6 / 3,7". The loop consumes 8 memd x 8B = 64 distinct
      // bytes/iteration and fetch_ptr advances exactly 64 B/iteration, one row ahead. Perturbing that rate loses
      // BOTH ways -- 32: +4.2% | 64: vendored | 96: +5.9% | 128: +12.3% -- a V with its minimum at the derived rate.
      //
      // MEASURED DEAD, each with its mechanism. Do not re-propose these:
      //   L2 BANDWIDTH / ARITHMETIC INTENSITY -- ⚠ THIS FILE ONCE ASSERTED "the conv ceiling is L2 read bandwidth,
      //     the lever is intensity". RETRACTED: 2 threads sustain 2.41 vrmpy/cycle = 154 B/cycle at d64's own
      //     intensity, scaling 2.02x linearly, and the HIGHER-intensity variant (32 B/vrmpy) is SLOWER at 2.30.
      //   PACKET DENSITY / ILP -- one thread issues ~1.2 vrmpy/cycle, so d64's 1.78-1.83 vrmpy/PACKET is already
      //     over-packed. A 10th packet is affordable (+0.68%, the loop is stall-bound) but has nothing to hold.
      //   A SECOND dcfetch STREAM -- impossible and pointless. Two in one packet is "invalid instruction packet";
      //     one in any compute packet is "out of slots" (each holds vmem + 2 vrmpy + memd). And a dcfetch that
      //     occupies its own issue slot costs about as much as the L1 miss it prevents: a bulk L1 pre-warm from C
      //     makes gvconv 9.2% faster and the whole model SLOWER at every dose.
      //   PER-CALL COST -- ~27 packets, not the ~2000 a structural fit suggested. Splitting the same work across
      //     2x the calls costs +0.05% (distill, +432 calls). So do NOT restructure around call count, and do not
      //     re-enable d32's disabled multi-row path (:290/:306) hoping to amortise it.
      //   L1 RESIDENCY of the circ buffer -- r(size, packets/vrmpy) = -0.168. The buffer is never L1-resident at
      //     ANY size, which is why size does not correlate; not because L1 does not matter.
      //   VERTICAL BLOCK FUSION (MNv2) -- per-row cost is FLAT across the L2 cliff (15.17/16.04/15.79/15.71
      //     us-row at 1176/588/294/147 KB), so these ops are not residency-bound and the traffic saved is bytes
      //     that are not costing time. See ~/.claude/plans/mnv2-vertical-fusion.md.
      PHASE(PH_ROW_US){
      int c=0;
      PHASE(PH_GV_US){
      if(oc & 1){   // odd out_chunks: func32 does the first 32-channel chunk
        gvconv2dbbb_circ_d32_v65_asm(cin, (const signed char*)gwt, orow,
          w->in_width_pad, w->out_next_row, w->Wo, w->stride_wh, w->Cigp, w->kw, w->kh, 1,
          gbias, mm, (const unsigned int*)grecip, w->Wop, w->circ, w->zshift, w->xzp, store_cntrl);
        c=1;
      }
      if(c < oc)    // func64: (oc-c)/2 chunk-PAIRS, 64 out channels each (one activation load, two weight sets).
                    // NOTE: d64's next_out_width arg is out_next_d32 in BYTES (Wop*32), not pixels (the driver passes
                    // out_next_d32 here vs out_next_d32>>5 for func32) -- it's the byte stride between the 2 output chunks.
        gvconv2dbbb_circ_d64_v65_asm(cin, (const signed char*)(gwt+(long)c*w_gcs), orow+(long)c*w->ochunk_w*32,
          w->in_width_pad, w->out_next_row, w->Wo, w->stride_wh, w->Cigp, w->kw, w->kh, (oc-c)/2,
          gbias+c*32, mm, (const unsigned int*)(grecip+c*32), w->ochunk_w*32, w->circ, w->zshift, w->xzp, store_cntrl);
      }
      PHASE(PH_REP_US) if(oy < w->oy1-1)
        repstream2_asm(w->inp+newrow*w->in_next_row, cin, w->Win, w->Cigp, n_in_rows, w->rpad_lpad, w->sw,
                       w->circ, w->buf_height, w->xzp, 8);
      }
      cbuf_row += w->sh; if(cbuf_row>=w->buf_height) cbuf_row -= w->buf_height;
    }
  }
  if(w->lock) qurt_hvx_unlock();
  return 0;
}
// ---- THE nnlib 1x1 GEMM PATH IS OBSOLETE, NOT MISSING. Deleted in 84a982b4e for having no emitter; the
// archaeology (2026-07-25) says WHY it had none, and it is not worth reviving:
//   * gemmpybbw wants NHWC [HW,Cin] as the matmul. EVERY 1x1 conv in BOTH models is d32-RESIDENT -- 36/36 in
//     distill, 34/34 in MNv2, zero plain NHWC. Routing them through it needs d32->NHWC in and NHWC->d32 out per
//     op, i.e. exactly the pack/from_d32 roundtrip d32 residency exists to remove (measured 22ms / 43% of
//     MobileNetV2 historically). The gemm predates residency; residency made it unreachable.
//   * It would not fix the memd stall anyway. gemmpybbw's inner loop is `vrmpy(y.ub, xNNxNN.ub)` with y from
//     vmem (weights) and x from memd (activations) -- 48 memd in the file. The 256-MAC vrmpy form REQUIRES a
//     scalar register-pair operand, so every kernel using it loads one side with scalar loads. gvconv and
//     gemmpybbw have the SAME activation-load structure; only which operand is vector differs.
// Sizing, for whoever asks again: 1x1 s1 convs are 38% of distill's conv time (2.320ms) and 100% of MNv2's
// (2.109ms) -- its 3x3s are all depthwise -- so the target is large. What a gemm would actually remove is the
// STAGING (repstream2 0.33ms + initial circ fill 0.26ms wall, across ALL convs), not the memd stream.
// REPAD one group: producer plain d32 [H][Cin/32][iwp][32] (chunks interleaved) -> per-group padded [Hp][Dg][Win][32]
// with xzp borders. Border-fill + one contiguous copy per row/chunk (NO transpose). noinline so conv_op's stack frame
// (mm[], the v65 locals) does NOT grow -> avoids a DSP stack-overflow rc=39 on stack-tight models (MobileNetV2).
__attribute__((noinline)) static void conv_repad_d32(unsigned char* dst, const unsigned char* in, int g, int Dg,
    int groups, int H, int W, int Hp, int Win, int ph, int pw, int xzp){
  // ★★★ THE REPAD IS UNALIGNED-STORE BOUND -- 0.855ms on distill (9.4% of the model) for 1.08MB, i.e. 1.26 GB/s
  // from fully vectorized helpers. pw is 1 for a 3x3, so the data lands 32 bytes into a 128-byte line and EVERY
  // vector store is unaligned. Forcing aligned stores measures 0.953 -> 0.514 ms, -46%.
  // ⚠ Two ablations that proved nothing, both instructive: skipping the side border fills saved only 12% (the
  // fills are not the cost); and passing an ALREADY-ALIGNED address changed nothing, because d32_copy takes
  // HVX_UVec* for both operands so clang emits vmemu regardless of the value. THE ALIGNMENT MUST BE IN THE TYPE.
  // ★★ TWO KERNEL-SIDE REWRITES FAILED; THE -46% IS ONLY REACHABLE BY REMOVING THE STRADDLES.
  //   (1) per-vector unaligned-load/aligned-store with a staging temp: +20% on distill.
  //   (2) the same, restructured branch-free -- ranges hoisted, at most two straddling vectors peeled outside a
  //       tight body, 8-byte tails on the peels: repad 0.851 -> 2.819 ms, THREE TIMES WORSE.
  // Why (2) still lost, and it is the useful part: the -46% measurement came from `d32_copy_a`, a plain aligned
  // vector loop with NO straddle handling at all. Real rows straddle at BOTH ends (lo = pw*32 = 32), and the two
  // peeled vectors cost more than the 15 aligned stores between them save. Every conv here has Win%4==0, so the
  // aligned path was taken on all 20 -- this is not a fallback artefact.
  // => The alignment win requires there to be NO straddle, i.e. lo must be a multiple of 128. That is the T1a
  // route: force the left pad to 4 pixels (padL_override=4, exactly as dw_geom already does and for exactly this
  // reason), so the valid region starts 128-aligned by construction and the copy is a plain aligned memcpy, with
  // in_left_skip absorbing the extra 3 pixels in the consumer. A GEOMETRY change, not a kernel change.
  int iwp=(W+3)&~3; long dstrow=(long)Dg*Win*32, srcrow=(long)groups*Dg*iwp*32;   // src row = ALL chunks (interleaved)
  d32_fill(dst, xzp, (long)ph*dstrow);                          // top pad rows
  d32_fill(dst+(long)(ph+H)*dstrow, xzp, (long)(Hp-ph-H)*dstrow);   // bottom pad rows
  // ★ ROLLING SOURCE PREFETCH. The repad's SOURCE is the producer's d32 output, which for a big early-stage
  // tensor (the stem's 1MB packed seed) has already been evicted -- so this loop was reading DDR at latency with
  // no request outstanding, the same defect the entry pack had. Two rows ahead is enough to cover the copy.
#ifndef REPAD_PF_ROWS
#define REPAD_PF_ROWS 2
#endif
  { unsigned nb=(unsigned)((srcrow*(REPAD_PF_ROWS<H?REPAD_PF_ROWS:H))/128); if(nb>65535) nb=65535;
    if(nb) l2f(in, 128, nb); }
  for(int h=0;h<H;h++){ unsigned char* row=dst+(long)(ph+h)*dstrow; const unsigned char* srow=in+(long)h*srcrow;
    if(h+REPAD_PF_ROWS<H){ unsigned nb=(unsigned)(srcrow/128); if(nb) l2f(srow+(long)REPAD_PF_ROWS*srcrow, 128, nb); }
    for(int d=0;d<Dg;d++){ unsigned char* ds=row+(long)d*Win*32;
      d32_fill(ds, xzp, (long)pw*32);
      // ALIGNED when the geometry gave this buffer padL=4 (pw*32 = 128) and W is a whole number of 4-pixel
      // groups -- both true for every spatial conv in distill. That is the entire point of the padL=4 geometry:
      // it removes the straddle so the copy is a plain aligned vector loop.
      if(!((pw*32)&127) && !((W*32)&127))
           d32_copy_a(ds+(long)pw*32, srow+(long)(g*Dg+d)*iwp*32, (long)W*32);
      else d32_copy(ds+(long)pw*32, srow+(long)(g*Dg+d)*iwp*32, (long)W*32);
      d32_fill(ds+(long)(pw+W)*32, xzp, (long)(Win-pw-W)*32); } }
}
// ★★★ THE ENTRY PACK IS PREFETCH-LESS, AND THAT -- NOT THE CONV -- IS THE WIDE-SEED COST.
// to_d32_asm reads the NHWC seed with UNALIGNED vector loads (4 overlapping vmemu per 4 pixels at depth 32) and
// issues NO l2fetch, so once the seed exceeds what L2 holds it runs at DDR LATENCY rather than bandwidth.
// Measured per-op (PROF_OPS) on identical-conv chains, 32ch 3x3 s1, H=64:
//     seed KB   128    256    512   1024
//     pack us    34    341   1370   2691      -> 3.8 / 0.75 / 0.37 / 0.39 GB/s
//     conv us   200    400    750    ~1500    -> 189 / 189 / 201 GMAC/s  (FLAT -- the conv has no width problem)
// The 128KB point is fast only because the entry `memcpy(arena+seed_off, seed, ...)` left it hot in L2.
// ⚠ A whole session read those pack times as CONV times and chased a "wide-row penalty in the conv" that does
// not exist: verify.py's PROF readout had `nops = size//48 - 2`, which shifted every label by one op AND
// dropped the last -- so the entry OP_PACK was printed under the first conv's name. The per-op array is now
// printed with its OPCODE (11=PACK, 1=CONV) so that cannot recur.
// FIX: convert in L2-sized ROW BLOCKS and l2fetch the next block while this one converts -- the same structure
// the conv worker already uses per row. The asm is unchanged; only the call is blocked.
// SWEPT (W=256 H=64 32ch, per-op us): the two transposes want DIFFERENT block sizes, so they get their own knob.
//   pack   KB   16   32   64  128  256(=unblocked-ish)      unpack KB   16   32   64  128
//   pack   us  135  212  368  819  1341                     unpack us ~1030 ~585 ~720 ~1150
// Smaller is better for the pack because the l2fetch has to LAND before the block is consumed; at 256KB it
// cannot, which is the original no-prefetch behaviour.
#ifndef PACK_BLK_KB
#define PACK_BLK_KB 16
#endif
#ifndef UNPACK_BLK_KB
#define UNPACK_BLK_KB 32
#endif
static void pack_op(unsigned char* in, int W, unsigned char* out, int onext, int H, int depth,
                    unsigned long long* prof){
  long inrow = (long)W*depth, outrow = (long)onext*(depth>>5);
  if(inrow<=0 || H<=0 || (depth&31)){ to_d32_asm(in, W, out, onext, H, depth); return; }
  int rows = (int)((PACK_BLK_KB*1024L)/inrow); if(rows<1) rows=1; if(rows>H) rows=H;
  { unsigned nb=(unsigned)((inrow*rows)/128); if(nb>65535) nb=65535; if(nb) L2FP(prof, in, 128, nb); }
  for(int y=0; y<H; y+=rows){
    int n = (y+rows<=H) ? rows : (H-y);
    if(y+rows<H){                                    // fetch the NEXT block before converting this one
      int m = (y+2*rows<=H) ? rows : (H-y-rows);
      unsigned nb=(unsigned)((inrow*m)/128); if(nb>65535) nb=65535;
      if(nb) L2FP(prof, in+(long)(y+rows)*inrow, 128, nb);
    }
    to_d32_asm(in+(long)y*inrow, W, out+(long)y*outrow, onext, n, depth);
  }
}
// The EXIT transpose has the identical defect and it is bigger: from_d32_asm is the graph's last conv's tail, and
// on a 512KB output it measured 1530 us (0.34 GB/s) -- twice the conv that produced it. Same blocking, prefetching
// the d32 SOURCE. (Only the graph's final conv unpacks; every intermediate is out_d32 resident.)
static void unpack_op(unsigned char* d32, int inext, unsigned char* out, int W, int H, int depth,
                      unsigned long long* prof){
  // Stride EXACTLY as the asm computes it (`in_width_depth = next_width_d32 * (depth>>5)`), and only block when
  // that is a faithful row stride -- a non-mult-32 depth is handed straight to the asm rather than guessed at.
  long inrow = (long)inext*(depth>>5), outrow = (long)W*depth;
  if(inrow<=0 || H<=0 || (depth&31)){ from_d32_asm(d32, inext, out, W, H, depth); return; }
  int rows = (int)((UNPACK_BLK_KB*1024L)/inrow); if(rows<1) rows=1; if(rows>H) rows=H;
  { unsigned nb=(unsigned)((inrow*rows)/128); if(nb>65535) nb=65535; if(nb) L2FP(prof, d32, 128, nb); }
  for(int y=0; y<H; y+=rows){
    int n = (y+rows<=H) ? rows : (H-y);
    if(y+rows<H){
      int m = (y+2*rows<=H) ? rows : (H-y-rows);
      unsigned nb=(unsigned)((inrow*m)/128); if(nb>65535) nb=65535;
      if(nb) L2FP(prof, d32+(long)(y+rows)*inrow, 128, nb);
    }
    from_d32_asm(d32+(long)y*inrow, inext, out+(long)y*outrow, W, n, depth);
  }
}

static void conv_op(unsigned char* in,unsigned char* out,unsigned char* d32in,unsigned char* d32out,
   unsigned char* weights,int* biasbuf,int* recip,int* minmax,int* p,unsigned long long* prof,
   unsigned char* arena,pool_t* pool){
  int Cin=p[P_Cin],H=p[P_H],W=p[P_W],Cout=p[P_Cout],kh=p[P_kh],kw=p[P_kw],sh=p[P_sh],sw=p[P_sw];
  int ph=p[P_ph],pw=p[P_pw],groups=p[P_groups];
  int xzp=p[P_xzp],zshift=p[P_zshift],Ho=p[P_Ho],Wo=p[P_Wo],Wop=p[P_Wop],Win=p[P_Win],Hp=p[P_Hp];
  int w_gcstride=p[P_w_gcstride];
  // Channel-group padding comes from the record (p[26..28] = rec[32..34]), derived once in geometry.conv_geom.
  // Verified on-device under -DGEOM_ASSERT (transitional, since removed) for all 34 MNv2 + 56 distill convs.
  int Cig=Cin/groups, Cog=Cout/groups, Cigp=p[P_Cigp], Cogp=p[P_Cogp];
  int out_chunks=Cogp/32, totchunks=p[P_tot];
  int in_gstride=Hp*(Cigp/32)*Win*32;
  int out_next_row=Wop*32*totchunks, stride_hw=(sh<<16)|sw;
  // d32-RESIDENCY: p[19]=in_d32 (input already d32 in `in` -> skip pack), p[20]=out_d32 (write d32 to `out` -> skip
  // from_d32). Eliminates the per-layer pack+from_d32 roundtrip (measured 22ms/43% of MobileNetV2).
  // Layout [Hp][Cigp/32][Win][32].
  int in_d32=p[P_in_d32], out_d32=p[P_out_d32];
  unsigned char* g_circ = arena + p[P_circ];   // V65 circular-buffer scratch (arena offset)
  // T1a bordered output: chunk width (Wp) of a downstream dw's [Hp][D][Wp][32] buffer, and the byte offset of the
  int b_chunk_w=p[P_bord_Wp], b_base_off=p[P_bord_base];
  int ochunk_w = b_chunk_w ? b_chunk_w : Wop;                          // per-chunk output width in pixels
  int onr_out  = b_chunk_w ? (totchunks*ochunk_w*32) : out_next_row;   // output row stride (bytes)
  // in_d32 REPAD (k>1): the producer wrote a PLAIN d32 [H][Cigp/32][iwp][32] (iwp=ru(W,4)); a spatial conv needs the
  // padded [Hp][Cigp/32][Win][32]. Repad = border-fill (xzp) + one contiguous d32 copy per row/chunk -- NO transpose,
  // reads the producer's L2-resident d32 (not NHWC from DDR). Cheaper than pack (the dw's ind path measured 1.9->0.2ms).
  // Skips the reduce's from_d32 AND this conv's pack. 1x1 in_d32 (ph=pw=0) reads `in` directly (no repad). d32in scratch.
  int repad = in_d32 && (ph>0 || pw>0);
  unsigned char* d32ip = (in_d32 && !repad) ? in : d32in;
  unsigned char* d32op = out_d32 ? out : d32out;
  for(int g=0; g<groups; g++){
    // NHWC input is the 1x1-s1-unpadded shape ONLY, where NHWC is already d32-contiguous so to_d32_asm applies
    // verbatim. lower.py asserts that shape at emit; every other NHWC input used to fall to the scalar pack().
    PHASE(PH_CV_PACK){
      if(!in_d32) to_d32_asm(in, W, d32ip, Win*32, H, Cigp);
      else if(repad) PHASE(PH_CV_REPAD)
        conv_repad_d32(d32ip+(long)g*in_gstride, in, g, Cigp/32, groups, H, W, Hp, Win, ph, pw, xzp);
    }
    // ---- V65 CIRCULAR DATAPATH -- the ONLY conv path. repstream shuffles each input row into a small circular
    // buffer; gvconv2dbbb_circ reads it with SIGNED weights, so the activation-zero-point correction folds into
    // biasbuf via gemsumb and there is no per-pixel `suma` pass. Per-row: conv all out chunks, then repstream the
    // next input row. Makes the memory-bound 1x1s compute-bound (the path that reached SNPE's 8.5ms and passed it).
    PHASE(PH_CV_COMPUTE){
      int in_right_padpad = 8*sw;
      // in_left_skip: the producer's out_left_pad (e.g. a resident s2 dw's =1). repstream reads ALIGNED vmem from
      // col 0 and valign-shifts by in_left_skip to land the valid data at circ-buf col 0 (op_supernode_new.c:670). NO
      // added copy, NO unaligned pointer. Passed to repstream2 packed with in_right_padpad (Q6_R_combine_RlRl).
      int in_lskip = p[P_in_left_skip];
      int in_width_pad = (Win - in_lskip + 3 + in_right_padpad) & ~3;
      int buf_height = kh>sh ? kh : sh;                  // =1 for 1x1 s1
      long buf_width = (long)in_width_pad*2*Cigp;
      long slice = (buf_width*buf_height + 127) & ~127L;  // per-thread circ slice (128-aligned)
      // GROUPED: offset input/weights/bias/recip/output by the group (g0=g*out_chunks). The
      // V65 path was written for groups==1; without these, groups>0 read group-0's weights+input -> wrong (cos 0.55).
      int g0 = g*out_chunks;
      v65work_t base = { d32ip+(long)g*in_gstride, d32op+b_base_off+(long)g0*ochunk_w*32, g_circ,
        weights+(long)g0*w_gcstride, biasbuf+g0*32, recip+g0*32,
        Win, Cigp, in_width_pad, buf_height, (Cigp/32)*Win*32, (sh<<16)|sw, (in_right_padpad<<16)|(in_lskip&0xffff), sw, sh, kh, kw,
        Wo, Wop, onr_out, out_chunks, w_gcstride, zshift, xzp, 0, Ho, 0, ochunk_w, 1, buf_width, prof };   // T1a bordered: base_off, onr_out, ochunk_w=Wp
      // 2-way row split through the pool whenever there is more than one output row. `pool_run2` runs both halves
      // serially by itself if the pool never came up, so that is the single-thread path too and needs no branch.
      // ★ SPLIT AXIS: output ROWS, not out-chunks -- TESTED 2026-07-25, the out-chunk split LOST: 9.94 -> 11.40ms
      // (+14.7%; cosine unchanged, so the split was functionally correct, just slower). Chunk-splitting gives each
      // thread a DISJOINT weight subset, but forces BOTH threads to stream the WHOLE input (2x repstream + 2x
      // input reads), and that costs more than the halved weight traffic buys. Row split stays. Do not re-propose.
      if(Ho < CONV_NT){ base.oy1 = Ho; v65_tile_worker(&base); }
      else {
        int rp = (Ho + CONV_NT - 1) / CONV_NT;
        v65work_t w0=base, w1=base;
        w0.oy0=0;  w0.oy1=rp;   w0.pf_wt=1;               // thread 0 alone prefetches the SHARED weight set
        w1.oy0=rp; w1.oy1=Ho;   w1.pf_wt=!V65_ONE_WT_PF;
        w1.circ = g_circ + slice;                          // each thread gets its own circular-buffer slice
        pool_run2(pool, v65_tile_worker, &w0, &w1);        // barrier (~us), worker holds HVX unit 1
      }
    }
  }
  // Cog%32==0 (NOT groups==1) is the real condition: d32out chunk gc=gi*out_chunks+cc holds channels gi*Cog+cc*32,
  // so when Cog is a whole number of 32-chunks that is exactly 32*gc -- a grouped conv's d32out is BYTE-IDENTICAL
  // to a groups==1 conv's and the HVX transpose applies unchanged. Testing `groups==1` instead needlessly sent all
  // 16 grouped convs down a scalar path at 4.90 ns/B against from_d32's 0.75. Cog%32==0 is now GUARANTEED by
  // lower.py (_pad32 pads Cout, and build_program asserts it for a grouped conv that must emit NHWC), so there is
  // no scalar twin here at all -- see _pad32's docstring for why padding the shape and using the HVX transpose
  // beats unpacking the unpadded shape slowly (MNv2: 5.49 ms vs 22.55 ms).
  PHASE(PH_CV_UNPACK) if(!out_d32) unpack_op(d32op, Wop*32, out, Wo, Ho, Cout, prof);   // HVX d32->NHWC transpose
}

static float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
static int roundi(float v){ return (int)(v>=0.0f?v+0.5f:v-0.5f); }

// ★ gap_accum_nhwc WAS DELETED 2026-07-26, MEASURED DEAD -- like the NHWC setail above, no gate model ever
// entered it (distill's 18 SE_GATEs are all d32-resident; MobileNetV2 has no SE block). Its one structural legacy
// is the LANE ORDER: it accumulated in the Q6_Wuh_vzxt_Vub -> Q6_Wuw_vzxt_Vuh {0,2,1,3} double-widen permutation,
// so se_gate's readback had to un-permute. gap_accum_d32 produces NATURAL channel order, so with the twin gone
// that whole un-permutation disappears from se_gate too -- the readback is now just acc[c].
static int dot_u8i8(unsigned char* a, signed char* w, int K){   // Σ a[c]*w[c], vrmpy 4-way u8*i8; K mult128 fast-path
  int nv=K/128, s=0; HVX_Vector acc=Q6_V_vzero();
  HVX_Vector* av=(HVX_Vector*)a; HVX_Vector* wv=(HVX_Vector*)w;
  for(int v=0;v<nv;v++) acc=Q6_Vw_vrmpyacc_VwVubVb(acc, av[v], wv[v]);
  for(int r=64;r>=4;r>>=1) acc=Q6_Vw_vadd_VwVw(acc, Q6_V_vror_VR(acc,r));   // in-register reduce (cf usum_u8),
  s=Q6_R_vextract_VR(acc,0);                                               // not a store-to-stack + 32 scalar loads
  for(int i=nv*128;i<K;i++) s+=a[i]*w[i];
  return s;
}
// d32 GAP: per-channel sum over the VALID H x Wo region of a d32 buffer [Ho][C/32][Wop][32]. Produces acc_chan[C] in
// NATURAL channel order (unlike gap_accum_nhwc's {0,2,1,3} lane order) so se_gate's d32 path reads it plainly. Per chunk
// we accumulate the same vzxt double-widen (matches gap_accum_nhwc's 128-lane acc), then a tiny 128-op scalar reduce
// folds the 4 pixel-slots per lane into the 32 channel sums using the widen's lane permutation.
static void gap_accum_d32(unsigned char* restrict in, int* restrict acc_chan, int C, int Ho, int Wo, int Wop, int* restrict atab){
  int nchunk=C/32; long rowstride=(long)nchunk*Wop*32; int wg=Wo>>2, wtail=Wo&3;
  // MEMORY ORDER [h][chunk][w] (chunk-major within a row) + per-row l2fetch. Chunk-outer strided across rows and STALLED
  // even under a whole-slice prefetch (~50µs/blk = the SE bottleneck, measured). Per-chunk 128-int acc table
  // (atab, nchunk*128) in the ARENA scratch -- a fixed 8KB stack array faulted (rc=39): se_gate already holds ~8KB. Reduce end.
  // ★ THE WIDENING CHAIN IS NOT THE COST -- this loop is MEMORY-bound, not issue-bound. It does 3 vzxt + 4 vadd
  // = 8 HVX ops per 128-byte block, so accumulating in u16 first (1 vzxt + 2 vadd, flushing to u32 before a lane
  // can exceed 255*257) should have cut it ~2.5x. Measured: se.gap 0.404 -> 0.403 ms. Nothing. Do not re-try.
  // (That attempt also overflowed the atab arena region -- cosine 0.65 -- so any retry needs its own scratch.)
  for(int i=0;i<nchunk*128;i++) atab[i]=0;
  unsigned pfh=(unsigned)rowstride/128;
  if(pfh) l2f(in,128,pfh);
  for(int h=0; h<Ho; h++){
    if(pfh && h+1<Ho) l2f(in+(long)(h+1)*rowstride,128,pfh);
    for(int ch=0; ch<nchunk; ch++){
      HVX_Vector* iv=(HVX_Vector*)(in + (long)h*rowstride + (long)ch*Wop*32);
      HVX_Vector* ap=(HVX_Vector*)(atab+ch*128);
      HVX_Vector a0=ap[0],a1=ap[1],a2=ap[2],a3=ap[3];
      for(int g=0; g<wg; g++){                                  // one 128-byte block = 4 pixels x 32 ch
        HVX_VectorPair hh=Q6_Wuh_vzxt_Vub(iv[g]);
        HVX_VectorPair l0=Q6_Wuw_vzxt_Vuh(Q6_V_lo_W(hh)), l1=Q6_Wuw_vzxt_Vuh(Q6_V_hi_W(hh));
        a0=Q6_Vw_vadd_VwVw(a0,Q6_V_lo_W(l0)); a1=Q6_Vw_vadd_VwVw(a1,Q6_V_hi_W(l0));
        a2=Q6_Vw_vadd_VwVw(a2,Q6_V_lo_W(l1)); a3=Q6_Vw_vadd_VwVw(a3,Q6_V_hi_W(l1));
      }
      ap[0]=a0; ap[1]=a1; ap[2]=a2; ap[3]=a3;
    }
  }
  for(int ch=0; ch<nchunk; ch++){ int* a128=atab+ch*128;
    for(int c=0;c<32;c++){   // reduce: Σ_slot a128[perm(slot*32+c)], perm(b)=(gg(b&3)<<5)+(b>>2), gg={0,2,1,3}
      int s=0;
      for(int slot=0;slot<4;slot++){ int b=slot*32+c, rr=b&3, gg=(rr==1?2:(rr==2?1:rr)); s+=a128[(gg<<5)+(b>>2)]; }
      acc_chan[ch*32+c]=s;
    }
    if(wtail){ unsigned char* p0=in + (long)ch*Wop*32;
      for(int c=0;c<32;c++){ int s=acc_chan[ch*32+c];
        for(int h=0;h<Ho;h++){ unsigned char* p=p0+(long)h*rowstride; for(int w=wg*4;w<Wo;w++) s+=p[w*32+c]; }
        acc_chan[ch*32+c]=s; } }
  }
}
// The residual SE-tail on a d32 buffer [Ho][C/32][Wop][32] (valid w<Wo). The per-channel gate
// tiles cleanly: a 128-byte block is 4 pixels x 32 ch, so the chunk's 32 gates (even->gqE low64, odd->gqO high64,
// repeated x4) match the vzxt lo/hi split and are reused across the chunk's rows.
// out_d32=1: write d32 [Ho][C/32][Wop][32] (resident-stage interior). out_d32=0: write NHWC [Ho*Wo][C] (stage exit ->
// the downsample/head packs it) -- the result vector is 4px x 32ch (d32 order), scattered as 4 x 32-byte NHWC stores.
// ---- ewise_blk: the elementwise CHAIN core --------------------------------------------------------------------
// ONE 128-byte block of the family every quantized tail belongs to:
//     out_u8 = sat_u8( ( sum_i f_i * (src_i[b] - zp_i) ) >> S  +  zo )
// The chain is a runtime-length step list, so SETAIL (gated conv + residual), a plain QLinearAdd, and any future
// n-term tail are the SAME code with a different `nst` -- that is the whole point of P3: a new architecture's
// elementwise op stops being new C.
//
// EVERY per-step constant is a VECTOR IN MEMORY, never a splat or a branch in here (learned the hard way,
// 2026-07-24): the first cut zero-initialised four accumulators and accumulated all steps, then vsplat'ed zp/f per
// block. That is 4 vzero + 4 vadd + 4 vsplat on top of ~25 real ops -- ~+48% ALU, measured +5.9% wall, which I
// wrongly blamed on "interpretation being too slow". It was the loop SHAPE. Hence:
//   * step 0 ASSIGNS, steps 1.. accumulate  (exactly what the hand-written kernel did)
//   * factors are `fv[0]`/`fv[1]` = the {even->lo, odd->hi} pair the Q6_Wuh_vzxt_Vub split wants. A per-tensor
//     scalar factor points at one pre-splatted pair; a per-channel gate points into the caller's gate table and
//     the CALLER advances `fv` per channel chunk. So addressing lives entirely in the caller and `restrict` lets
//     the invariant loads hoist to whatever loop level the caller implies -- the hand-hoisting, done by the
//     compiler instead of by us.
// Bit-exact with the kernels it replaces: same widen, same i16xi16->i32 vmpy, same i32 accumulate, same
// vasr_rnd_sat requant, same order.
// `fv` points AT the pair for the block about to be computed -- when a factor is per-channel and the channel block
// is the caller's inner loop, the CALLER advances it (one pointer add). Keeping an `fstep*b` in here instead cost a
// measured +0.35% on distill: it is a load-multiply-add per step per block that survives even when the stride is 0.
// Addressing belongs to the addressing loop; this core does arithmetic only.
typedef struct { const unsigned char* src; const HVX_Vector* zpv; const HVX_Vector* fv; } ewstep_t;
// PHASE 4 / E4.3: `:nt` (non-temporal) LOADS on the elementwise sources. Elementwise inputs are usually consumed
// once and dead -- a setail's conv and residual, an add's two operands -- so letting them allocate L2 evicts the
// NEXT op's weights for nothing. `:nt` marks them so they do not displace. Aligned loads only: vmemu has no :nt
// form, so the NHWC (ua=1) path keeps the ordinary load. This is the blunt version -- every elementwise source,
// not just the ones a lifetime map proves dead -- which is the right screening shape: if it does nothing here, a
// per-buffer last-use map cannot do better.
#ifndef NT_LOADS
#define NT_LOADS 0
#endif
static inline HVX_Vector vldnt(const void* p){ HVX_Vector v; asm volatile("%0 = vmem(%1+#0):nt":"=v"(v):"r"(p)); return v; }

// `ua`: unaligned source loads (vmemu). NHWC tensors have an arbitrary channel count so their 128-byte blocks are
// not 128-aligned; d32 blocks always are. A literal at every call site, so the select folds away.
__attribute__((always_inline)) static inline void ew_mul(const ewstep_t* s, long b, int ua, HVX_VectorPair* ml, HVX_VectorPair* mh){
  HVX_Vector zp=s->zpv[0], v= ua ? ((const HVX_UVec*)s->src)[b]
      : (NT_LOADS ? vldnt((const HVX_Vector*)s->src + b) : ((const HVX_Vector*)s->src)[b]);
  HVX_VectorPair p=Q6_Wuh_vzxt_Vub(v);
  *ml=Q6_Ww_vmpy_VhVh(Q6_Vh_vsub_VhVh(Q6_V_lo_W(p),zp),s->fv[0]);
  *mh=Q6_Ww_vmpy_VhVh(Q6_Vh_vsub_VhVh(Q6_V_hi_W(p),zp),s->fv[1]);
}
// `vqm` clamps to qmax before the u8 pack; pass clampq=0 (a literal at every call site, so it folds) when qmax==255.
__attribute__((always_inline)) static inline HVX_Vector ewise_blk(const ewstep_t* restrict st, int nst, long b,
    HVX_Vector vzo, int S, int clampq, HVX_Vector vqm, int ua){
  HVX_VectorPair ml,mh; ew_mul(st,b,ua,&ml,&mh);
  HVX_Vector alo=Q6_V_lo_W(ml), ahi=Q6_V_hi_W(ml), blo=Q6_V_lo_W(mh), bhi=Q6_V_hi_W(mh);
  for(int i=1;i<nst;i++){ ew_mul(st+i,b,ua,&ml,&mh);
    alo=Q6_Vw_vadd_VwVw(alo,Q6_V_lo_W(ml)); ahi=Q6_Vw_vadd_VwVw(ahi,Q6_V_hi_W(ml));
    blo=Q6_Vw_vadd_VwVw(blo,Q6_V_lo_W(mh)); bhi=Q6_Vw_vadd_VwVw(bhi,Q6_V_hi_W(mh)); }
  HVX_Vector rlo=Q6_Vh_vasr_VwVwR_rnd_sat(ahi,alo,S), rhi=Q6_Vh_vasr_VwVwR_rnd_sat(bhi,blo,S);
  rlo=Q6_Vh_vadd_VhVh(rlo,vzo); rhi=Q6_Vh_vadd_VhVh(rhi,vzo);
  if(clampq){ rlo=Q6_Vh_vmin_VhVh(rlo,vqm); rhi=Q6_Vh_vmin_VhVh(rhi,vqm); }
  return Q6_Vub_vasr_VhVhR_rnd_sat(rhi,rlo,0);            // u8 saturation is the relu (zo==0)
}
static void setail_d32(unsigned char* restrict out, unsigned char* restrict conv, unsigned char* restrict res, float* gate, float* blob,
   int C,int Ho,int Wo,int Wop,int zc,int zr,int zo,int relu,int hasres,int has_gate,int out_d32, unsigned char* scratch){
  float Asc=blob[0], Asr=blob[1]; float mx=Asc>Asr?Asc:Asr; if(mx<1.0f) mx=1.0f;
  int S=15; while(mx*(float)(1<<S) >= 32760.0f) S--;
  int rf=roundi(Asr*(float)(1<<S));
  // ewise_blk's constants live in MEMORY, not registers: kv = {zp_conv, zp_res, f_res, f_res} -- the residual's
  // per-tensor factor occupies a PAIR because a step's fv[0]/fv[1] are the even/odd halves of the vzxt split, and
  // a scalar factor is simply the same splat in both. The gated conv step points straight into gqtab instead.
  HVX_Vector kv[4] __attribute__((aligned(128)));
  kv[0]=Q6_V_vsplat_R(((zc&0xffff)<<16)|(zc&0xffff));
  kv[1]=Q6_V_vsplat_R(((zr&0xffff)<<16)|(zr&0xffff));
  kv[2]=kv[3]=Q6_V_vsplat_R(((rf&0xffff)<<16)|(rf&0xffff));
  HVX_Vector vzo=Q6_V_vsplat_R(((zo&0xffff)<<16)|(zo&0xffff));
  int nchunk=C/32; long rowstride=(long)nchunk*Wop*32; int wg=Wo>>2, wtail=Wo&3;
  // Precompute ALL chunk gates once (even->gqtab[ch][0..63], odd->[64..127], tiled x4) so the hot loop is contiguous
  // in MEMORY ORDER [h][chunk][w] -- the d32 buffer is chunk-major within a row, so h-outer/chunk-inner streams
  // sequentially (chunk-outer strided across rows -> DDR thrash, the first cut's 3.6ms regression).
  // ★ gqtab lives in the SHARED SCRATCH, not on the stack. It was `short gqtab[2048]` -- a fixed 4 KB frame sized
  // for <= 16 chunks (512ch), because an alloca faulted the DSP stack. But it is indexed gqtab[ch*128 + k] for
  // ch < C/32, i.e. it needs C*4 SHORTS: C=512 filled it exactly, C=640 ran 1 KB PAST it (silently corrupting the
  // stack -- and every RepViT measurement taken before 2026-07-28 was doing exactly that), and C=768 ran 2 KB past.
  // The fault surfaces as PermissionError on the FastRPC invoke, which exec_lib masks behind a reset whose own
  // teardown throws "ioctl returned 39" pointing at init_dsp -- a symptom with no visible relation to channel
  // width, which is what made this expensive to find.
  short* gqtab = SCR_TAKE(scratch, C*8);
  for(int ch=0; ch<nchunk; ch++) for(int k=0;k<64;k++){
    float ge=has_gate?gate[ch*32+2*(k&15)]:1.0f, go=has_gate?gate[ch*32+2*(k&15)+1]:1.0f;
    gqtab[ch*128+k]=(short)roundi(Asc*ge*(float)(1<<S)); gqtab[ch*128+64+k]=(short)roundi(Asc*go*(float)(1<<S));
  }
  unsigned pfh=(unsigned)rowstride/128;   // prefetch a full row ahead: setail is ~memory-stall bound (no HW prefetcher)
  if(pfh){ l2f(conv,128,pfh); if(hasres) l2f(res,128,pfh); }
  for(int h=0; h<Ho; h++){
    if(pfh && h+1<Ho){ l2f(conv+(long)(h+1)*rowstride,128,pfh); if(hasres) l2f(res+(long)(h+1)*rowstride,128,pfh); }
    for(int ch=0; ch<nchunk; ch++){
      unsigned char* pc=conv+(long)h*rowstride+(long)ch*Wop*32;
      unsigned char* pr=hasres?res+(long)h*rowstride+(long)ch*Wop*32:pc;
      unsigned char* pod=out+(long)h*rowstride+(long)ch*Wop*32;        // d32 out position
      unsigned char* pon=out+(long)h*Wo*C+ch*32;                       // NHWC out position (row base, +w*C below)
      HVX_Vector* o=(HVX_Vector*)pod;
      // the chain, rebuilt per chunk -- only the gate pointer actually moves, which is why the caller owns it
      ewstep_t st[2]={{pc,kv+0,(const HVX_Vector*)(gqtab+ch*128)},{pr,kv+1,kv+2}};
      for(int g=0; g<wg; g++){
        HVX_Vector rv=ewise_blk(st,hasres?2:1,g,vzo,S,0,vzo,0);        // 4px x 32ch (d32 order)
        if(out_d32) o[g]=rv;
        else { unsigned char tb[128] __attribute__((aligned(128))); *(HVX_Vector*)tb=rv;   // scatter to NHWC: 4 x 32B
          for(int p=0;p<4;p++) __builtin_memcpy(pon+(long)(g*4+p)*C, tb+p*32, 32); }
      }
      for(int wp=wg*4; wp<Wo; wp++) for(int l=0;l<32;l++){       // tail cols (Wo not mult-4): scalar
        int c=ch*32+l; float v=Asc*(has_gate?gate[c]:1.0f)*((float)pc[wp*32+l]-zc);
        if(hasres) v+=Asr*((float)pr[wp*32+l]-zr);
        if(relu&&v<0.0f) v=0; int q=roundi(v)+zo; if(q<0)q=0; if(q>255)q=255;
        (out_d32?pod:pon)[out_d32? wp*32+l : (long)wp*C+l]=(unsigned char)q;
      }
    }
  }
}
static void se_gate(float* gate, unsigned char* expand, signed char* fc1w, signed char* fc2w,
   float* blob, float* lut, int Cexp,int Csq, int Ho,int Wo,int Wop, unsigned char* scratch,
   unsigned long long* prof){
  float g0=blob[0],g1=blob[1],inv_srm=blob[2],z_rm=blob[3];
  float* M1=blob+4; float* b1q=M1+Csq; float* W1=b1q+Csq;   // W1 = Sigma w per fc1 out (vrmpy zp correction)
  // Cexp need only be a multiple of 32; fc2 and the sigmoid run in whole 128-lane blocks, so the weights and the
  // mAi/mBi tables are padded to this stride host-side (lower.py) and the tail block's pad lanes are simply never
  // read back. The padding is also what keeps each weight ROW 128-aligned for dot_u8i8's aligned vector loads.
  int Cexpp=(Cexp+127)&~127;
  int* mAi=(int*)(W1+Csq); int* mBi=mAi+Cexpp; int Ssig=*(int*)(mBi+Cexpp);   // fixed-point sigmoid idx (see lower.py)
  l2f(lut, 128, 128);   // 16KB sigmoid LUT -> L2, so the data-dependent gather below hits cache not DDR
  { unsigned h1=(unsigned)(Csq*Cexpp)/128; if(h1){ l2f(fc1w,128,h1); l2f(fc2w,128,h1); } }   // fc1/fc2 weights -> L2 (no L2 hw prefetcher)
  // ALL FOUR working arrays come out of the shared scratch (see SCR_TAKE), in this order -- lower.py's SE emitter
  // declares exactly these four sizes. They were on the stack, bounded by a host-side `Cexpp <= 1536` build error
  // that covered acc/xqu and SILENTLY NOT fc1u: fc1u[128] is indexed to Csq, so a 768-channel SE (Csq=192) wrote
  // 256 B past it into interp's frame and faulted the PD. Off the stack, se_gate has no width ceiling at all --
  // and interp's frame drops ~8 KB, which is half the thread's whole stack.
  int* atab = SCR_TAKE(scratch, Cexp / 32 * 128 * 4);      // gap_accum_d32's per-chunk 128-int accumulator table
  int* acc = SCR_TAKE(scratch, Cexpp * 4);                 // GAP sums -> requant -> reused as fc2's acc2 (Cexpp lanes)
  unsigned char* xqu = SCR_TAKE(scratch, Cexp);            // u8 squeeze input for vrmpy fc1
  float* fc1u = SCR_TAKE(scratch, Csq * 4);
  // ★ Cexp is a multiple of 32 (_pad32) and the weight tables are padded to a multiple of 128 (lower.py), so every
  // loop below is fully vectorized with no tail and no scalar twin. This function used to carry three scalar
  // fallbacks for non-mult-128 widths -- a scalar GAP, a scalar per-channel requant and a scalar fc2 -- none of
  // which any model has ever executed. They were the silent-slow-path shape C5's fast-or-fail policy exists to
  // refuse: an architecture that hit one would have run correctly and ~13x slow with nothing to say so. A width
  // that needs them needs a VECTORIZED mult-32 path, which is real work and belongs to MobileNetV3 support
  // (task #59), not a fallback that makes the failure invisible.
  PHASE(PH_SE_GAP) gap_accum_d32(expand, acc, Cexp, Ho, Wo, Wop, atab);
  // ★ VECTOR GAP REQUANT. q = clamp(round(A*gsum - Bf) + z_rm, 0, 255) with A = g0*inv_srm and Bf = g1*inv_srm.
  // Both are PER-TENSOR scalars out of blob[0..3], so this is a splat, not a per-channel table -- and it is the
  // same integerize-the-affine move the sigmoid below already makes, for the same reason: HVX v65 has no float
  // vector unit, so a per-channel float chain cannot vectorize at all. Measured worth: ablating the scalar loop
  // was -2.80% on distill (5/5 paired), so unlike the T1a border zap this is real REMOVABLE work.
  //
  // Two things this deliberately does NOT do, both lane-order traps that the sigmoid already navigates:
  //  * it does NOT pack to u8 in vector form. `acc` is NATURAL order under d32 but the vzxt double-widen order
  //    for NHWC, and the {0,2,1,3} unswizzle lives in the readback loop below. Rewriting acc IN PLACE with pure
  //    ELEMENTWISE int32 ops is order-agnostic, so ONE path serves both and the readback is untouched.
  // S=21 is safe for ANY shape: Ai ~ 2^21/HW while gsum <= 255*HW, so |Ai*gsum| <= 255*2^21 and |Bi| the same,
  // leaving |p| <= 1.07e9 < 2^31. The ewise core's S=15 would give Ai=16 at HW=2048 -- 3% steps in A, i.e. up to
  // 8 quantization levels of error.
  // ★★ EDITING THIS FUNCTION PERTURBS MODELS THAT NEVER CALL IT. Measured for this change: the .so is EXACTLY
  // the same size (173452 B both ways) but 33% of its bytes differ, spanning almost the whole text section -- so
  // the compiler re-laid-out everything downstream. Consequence, both paired and consistent: MNv2 -0.75% (6/6
  // pairs) and the conv2x2s2 probe +2.38% (0/6), and NEITHER model contains an se_gate op. Opposite directions on
  // code that never executes is the signature of INSTRUCTION-side layout (I-cache sets, HVX packet alignment) --
  // it is not L2 data eviction, which could not touch a model that never runs this, and it is not ".so got
  // bigger", which is simply false. Budget +-2% of unrelated movement whenever megakernel.c changes shape.
  // ★ Which raises the obvious question about the win below: is IT layout too? No -- ATTRIBUTE BY OP. Per-op
  // profile, old binary vs new: the 18 se_gate ops went 1.040 -> 0.834ms (-19.8%) while the other 75 ops went
  // 8.260 -> 8.249ms (-0.13%, i.e. nothing), and the six biggest movers are all se_gate. Layout perturbs EVERY
  // op a little; it cannot concentrate a 20% drop onto exactly the ops running the rewritten loop.
  PHASE(PH_SE_RQ){
    // ★★ THE ZERO POINT IS SUBTRACTED FROM THE SUM, NOT FOLDED INTO A CONSTANT -- FIXED 2026-08-17.
    // This was  p = Ai*gsum + (2^(S-1) - Bi)  with  Bi = roundi(g1*inv_srm*2^S) = (sc_exp/s_rm)*zc_exp*2^S,
    // at a fixed S=21. That constant OVERFLOWS int32 as soon as (sc_exp/s_rm)*zc_exp > 1024, i.e. sc_exp/s_rm
    // > 8 at the usual zc_exp=128 -- and `roundi` is a bare float->int cast, so it SATURATES at INT_MAX
    // instead of wrapping, which destroys the two's-complement cancellation the expression depended on.
    // Every squeeze value in that block is then wrong, and the block still emits a perfectly plausible
    // sigmoid in [0,1], so nothing downstream can notice.
    // ★ MEASURED: the August student overflows 2 of its 18 SE gates (sc_exp/s_rm = 11.73 and 11.07) and read
    // 0.9523 device-vs-golden; the July one peaks at 6.36 = 80% of INT_MAX and read 0.9999332. That 20%
    // margin is the only reason this shipped. The old "S=21 is safe for ANY shape" note bounded |Ai*gsum|
    // correctly and then asserted |Bi| had the same bound -- false: Bi scales with sc_exp/s_rm, which is a
    // CALIBRATION ratio (how much wider the expand tensor is than its own spatial mean) with no a-priori
    // limit. It grows with heavy-tailed activations, e.g. anything trained with --wd-exclude-1d.
    // Since Bi/Ai == g1/g0 == zc_exp*HW EXACTLY, folding it back into the sum removes the constant outright:
    //     p = Ai*(gsum - zc_exp*HW) + 2^(S-1)
    // and |gsum - zc_exp*HW| <= 255*HW, which always fits.
    const int HW = Ho*Wo;                        // == lower.py's HW = prod(expand.shape[1:]), by construction
    int zc = roundi(g1/(g0*(float)HW));          // = zc_exp; <= 255, so the float divide rounds exactly
    int zsum = zc*HW;                            // exact in int32
    // The MULTIPLY still has to stay in range: |Ai*d| < 2^31 where d = gsum-zsum, |d| <= 255*HW. Rather than
    // lower S (which costs precision on models that were always fine -- a first cut did that and moved the
    // July gate by -0.000012), CLAMP d to +-lim = (2^31-1)/Ai first. That is EXACTLY equivalent, not an
    // approximation: |d| > lim implies |h| >= 2^(31-S) = 1024, i.e. far outside the [0,255] the result is
    // clamped to anyway, so both paths pin to the same rail. S therefore stays 21 for every shape and this
    // whole change is bit-identical wherever the old code did not overflow.
    enum { S = 21 };
    int Ai = roundi(g0*inv_srm*(float)(1<<S));
    float limf = 2147000000.0f/(float)(Ai > 1 ? Ai : 1);   // no libm here (-ffreestanding): plain compare
    int lim = (int)(limf > 2.0e9f ? 2.0e9f : limf);
    HVX_VecW vA = (HVX_VecW)Q6_V_vsplat_R(Ai), vB = (HVX_VecW)Q6_V_vsplat_R(1<<(S-1));   // +half = round-to-nearest
    HVX_Vector vZ = Q6_V_vsplat_R(zsum), vL = Q6_V_vsplat_R(lim), vNL = Q6_V_vsplat_R(-lim);
    HVX_Vector vzr = Q6_V_vsplat_R((int)z_rm), vlo = Q6_V_vzero(), vhi255 = Q6_V_vsplat_R(255);
    HVX_Vector* av = (HVX_Vector*)acc;
    for(int v=0; v<Cexp/32; v++){
      HVX_Vector d = Q6_Vw_vsub_VwVw(av[v], vZ);
      d = Q6_Vw_vmin_VwVw(Q6_Vw_vmax_VwVw(d, vNL), vL);
      HVX_VecW p = (HVX_VecW)d*vA + vB;
      HVX_Vector h = Q6_Vw_vadd_VwVw(Q6_Vw_vasr_VwR((HVX_Vector)p, S), vzr);
      av[v] = Q6_Vw_vmin_VwVw(Q6_Vw_vmax_VwVw(h, vlo), vhi255);
    }
    for(int c=0;c<Cexp;c++) xqu[c] = (unsigned char)acc[c];   // gap_accum_d32 gives NATURAL channel order
  }
  PHASE(PH_SE_FC1) for(int o=0;o<Csq;o++){             // fc1: vrmpy int acc -> requant relu -> u8
    float a=(float)dot_u8i8(xqu, fc1w+(long)o*Cexpp, Cexp) - z_rm*W1[o];  // Σ(xq-z_rm)*w; row stride is PADDED
    fc1u[o]=clampf((float)roundi((a+b1q[o])*M1[o]),0,255);
  }
  int* acc2=acc;                                        // reuse GAP accumulator (free after gap loop) -> no extra stack
  int nbk=Cexpp/128;
  PHASE(PH_SE_FC2){                                       // fc2 output-parallel: acc2[c]=Σ_o fc1u[o]*w2T[o][c] (fc2w=[Csq][Cexp])
    HVX_Vector* a2=(HVX_Vector*)acc2;
    for(int v=0;v<nbk*4;v++) a2[v]=Q6_V_vzero();
    for(int o=0;o<Csq;o++){
      HVX_Vector* wv=(HVX_Vector*)(fc2w+(long)o*Cexpp);
      unsigned f=((unsigned)(int)fc1u[o])*0x01010101u;    // vmpyi(Vu.w,Rt.b) applies Rt.b per word (i mod 4) -> replicate to all 4 bytes
      for(int blk=0;blk<nbk;blk++){
        HVX_VectorPair wh=Q6_Wh_vsxt_Vb(wv[blk]);
        HVX_VectorPair w0=Q6_Ww_vsxt_Vh(Q6_V_lo_W(wh)), w1=Q6_Ww_vsxt_Vh(Q6_V_hi_W(wh));
        a2[blk*4+0]=Q6_Vw_vmpyiacc_VwVwRub(a2[blk*4+0],Q6_V_lo_W(w0),f);
        a2[blk*4+1]=Q6_Vw_vmpyiacc_VwVwRub(a2[blk*4+1],Q6_V_hi_W(w0),f);
        a2[blk*4+2]=Q6_Vw_vmpyiacc_VwVwRub(a2[blk*4+2],Q6_V_lo_W(w1),f);
        a2[blk*4+3]=Q6_Vw_vmpyiacc_VwVwRub(a2[blk*4+3],Q6_V_hi_W(w1),f);
      }
    }
    // Sigmoid index in INT32 (was a per-channel scalar-float chain, ~0.46ms; HVX v65 has no float vector unit so
    // the affine had to be integerized). idx = (mAi*acc2 + mBi) >> Ssig, clamped [0,4095]; mAi/mBi/Ssig baked in
    // acc2's lane order at emit. Overwrites acc2 in place, then the SAME {0,2,1,3} readback gathers gate[c]=lut[idx].
    // mAi/mBi live inside the float-packed blob at a non-128-aligned offset, so they MUST be loaded unaligned
    // (HVX_VecWU -> vmemu); acc2 is aligned(128). An aligned load of mAi reads the wrong address and scrambles gate.
    HVX_Vector vhi=Q6_V_vsplat_R(4095), vz=Q6_V_vzero();
    HVX_VecWU* mav=(HVX_VecWU*)mAi; HVX_VecWU* mbv=(HVX_VecWU*)mBi;
    for(int v=0;v<nbk*4;v++){
      HVX_VecW p=(HVX_VecW)((HVX_Vector*)acc2)[v] * mav[v] + mbv[v];
      HVX_Vector idxv=Q6_Vw_vmin_VwVw(Q6_Vw_vmax_VwVw(Q6_Vw_vasr_VwR((HVX_Vector)p,Ssig),vz),vhi);
      ((HVX_Vector*)acc2)[v]=idxv;
    }
    for(int c=0;c<Cexp;c++){
      int l=c&127,rr=l&3,gg=(rr==1?2:(rr==2?1:rr)); gate[c]=lut[acc2[(c&~127)+(gg<<5)+(l>>2)]];
    }
  }
}
// ★ THE NHWC SETAIL (setail_hvx + its 2-way threaded wrapper) WAS DELETED 2026-07-26, MEASURED DEAD.
// Neither gate model ever entered it: distill's 17 SETAILs are all d32-resident, and MobileNetV2 has no SETAIL at
// all (its residuals are plain QLinearAdds, which go through ewise_op). setail_d32 already writes EITHER layout --
// out_d32=0 scatters 4x32B to NHWC -- so the only thing the deleted twin added was NHWC *input*, and after _pad32
// a SETAIL's producers are d32 by construction. lower.py now fails the build for a non-d32 SETAIL, naming the op.
// This is the one deletion in this campaign that removes CAPABILITY rather than a slower duplicate: there is no
// faster alternative, the alternative is a build error. It is justified the same way the dead V60 conv path was --
// an untested path is not genericity, it is the appearance of it, and this file has twice shipped a silent wrong
// answer down a branch no gate model ran (the nnlib stem record, the GAP-commute walk).
// Two measurements from its body worth keeping, because they still describe setail_d32:
//   * setail is 96% MEMORY STALL -- 5.31MB in 6.03ms = 0.88 GB/s against a 0.24ms compute floor. Hexagon's L2 has
//     no hardware prefetcher for streaming data, which is why the l2fetch a row ahead is load-bearing.
//   * ALIGNMENT DOES NOT MATTER for it: 1.127ms aligned vs 1.130ms forced-unaligned over 17 ops. Alignment pays
//     for STORE-BOUND COPIES (conv_repad_d32 got -35%), not for memory-bound elementwise.
// ---- OP_EWISE: the FLAT addressing loop over ewise_blk. --------------------------------------------------------
// Streams n contiguous bytes with per-tensor scales, so the pixel structure is irrelevant. The other two addressing
// loop over the same core is setail_d32 (d32 chunks); between them they cover every elementwise tail we have,
// which is the point -- the arithmetic exists ONCE and the layouts are thin loops.
//
// The bet this rests on: elementwise ops are MEMORY-bound and already at their floor (real SETAIL ~0.1ms of
// distill's 11.1ms; the ILP-unroll experiment 1fb9476fa proved adding ALU work changes nothing), so the per-vector
// step loop is free -- its cost hides under the memory stall. Confirmed by A/B against the old hand-written add.
#define EW_MAXST 4                                         // longest chain we build; raise with the callers, not here
typedef struct { unsigned char *src; int zp, f; } ewacc_t;
static void ewise_op(unsigned char* restrict out, const ewacc_t* st, int nst, int S, int zo, int qmax, long n){
  HVX_Vector vzo=Q6_V_vsplat_R(((zo&0xffff)<<16)|(zo&0xffff)), vqm=Q6_V_vsplat_R(((qmax&0xffff)<<16)|(qmax&0xffff));
  int clampq=(qmax<255);
  // Per-step constants go to a memory table ONCE (kv[3i]=zp, kv[3i+1..2]=the factor pair), not to a vsplat inside
  // the block loop. Flat per-tensor scales -> fstep 0, so every one of these loads hoists out of the loop.
  HVX_Vector kv[3*EW_MAXST] __attribute__((aligned(128))); ewstep_t es[EW_MAXST];
  for(int i=0;i<nst;i++){
    kv[3*i]=Q6_V_vsplat_R(((st[i].zp&0xffff)<<16)|(st[i].zp&0xffff));
    kv[3*i+1]=kv[3*i+2]=Q6_V_vsplat_R(((st[i].f&0xffff)<<16)|(st[i].f&0xffff));
    es[i]=(ewstep_t){st[i].src, kv+3*i, kv+3*i+1};
  }
  long nv=n/128; HVX_Vector* ov=(HVX_Vector*)out;
  const long CB=32;                                        // 4KB prefetch chunk (no L2 hw prefetcher)
  if(nv) for(int i=0;i<nst;i++) l2f(st[i].src,128,(unsigned)(nv<CB?nv:CB));
  for(long v=0;v<nv;v++){
    if((v%CB)==0 && v+2*CB<nv) for(int i=0;i<nst;i++) l2f(st[i].src+(v+2*CB)*128,128,CB);
    ov[v]=ewise_blk(es,nst,v,vzo,S,clampq,vqm,0);
  }
  for(long i=nv*128;i<n;i++){ long long acc=0;
    for(int k=0;k<nst;k++) acc+=(long long)(st[k].src[i]-st[k].zp)*st[k].f;
    long long q=((acc+(1LL<<(S-1)))>>S)+zo; if(q<0)q=0; if(q>qmax)q=qmax; out[i]=(unsigned char)q; }
}
// requant one 32-lane int32 accumulator (already = acc+bias) EXACTLY like reference.requant_ref:
//   out = clip_u8( ((s<<zsh)*recip + 2^30) >> 31 )  -- via the vmpye/vmpyo Q31 multiply-high (the SAME sequence
// nnlib's inconv332 uses). recip.uh/.h are the low/high halfwords of the per-oc recip; :<<1:rnd:sat = round((t*recip)/2^31).
static inline HVX_Vector requant_vec(HVX_Vector s, HVX_Vector recip, int zsh){
  HVX_Vector t = zsh ? Q6_Vw_vasl_VwR(s, zsh) : s;
  HVX_Vector y = Q6_Vw_vmpye_VwVuh(t, recip);
  y = Q6_Vw_vmpyoacc_VwVwVh_s1_rnd_sat_shift(y, t, recip);   // (t*recip)>>31 round-half-up, == requant_ref
  y = Q6_Vw_vmax_VwVw(y, Q6_V_vzero());
  return Q6_Vw_vmin_VwVw(y, Q6_V_vsplat_R(255));
}
// PIXELS-IN-LANES stem (STEM_PIL, op[16]==2). Reuses the vrmpy-stem weights (wi8 [tap][32ch][4 i8]). 32 output
// PIXELS in the lanes; per tap the input is a contiguous RGBA vector (gathered stride-2 from a small L1 cp4 row
// buffer), weights are i8 SCALAR-Rt (Q6_Vw_vrmpyacc_VwVubRb, hw-broadcast) -> NO vsplat -> X-units fed (269 GMAC/s
// validated). v1: scalar cp4 fill + scalar strided store (CORRECTNESS-FIRST; the vrmpy compute is the validated part;
// fill/store get vectorized once bit-exact). cp4 on the worker stack (3*WP*4 = 3456B < 16KB FastRPC stack).
// WP_PIL (padded cp4 gather width, RGBA words) arrives as a -D from lower.py, which sizes the cp4 ARENA from
// the SAME constant. Defining it here too is exactly how those two drift apart -- so require it instead.
#ifndef WP_PIL
#error "WP_PIL must be -D'd in from lower.py (it also sizes the cp4 arena there)"
#endif
// Fill ONE cp4 tap row from input row iy (copy3to4 vdelta + pad zeroing). Extracted so idea-D row-pairing can fill a
// 5-row window and share the middle input row (2*oy+1) between output rows oy and oy+1 instead of re-converting it.
// Fill ONE cp4 tap row from input row iy (copy3to4 vdelta + border zeroing). Extracted into a helper (with the
// output-row compute below) so the stem is one clean per-row pipeline.
static inline void stem_pil_fill(unsigned char* crow, unsigned char* in, int iy, int H, int W, int Cin, int pad, int xzp){
  // Fill the whole row with the activation zero-point (xzp): border/pad pixels then contribute (xzp-xzp)*w=0 once the
  // bias absorbs -xzp*Sigma_w (codegen). xzp==0 -> plain zero. The 4th RGBA byte is multiplied by weight[3]=0 (harmless).
  HVX_Vector fv = Q6_V_vsplat_R((unsigned)(xzp & 0xff) * 0x01010101u);
  for(int i=0; i<WP_PIL*4/128; i++) ((HVX_Vector*)crow)[i] = fv;   // also primes write-allocate (WP_PIL*4 is mult-128)
  if(iy>=0 && iy<H){ unsigned char* irow = in + (long)iy*W*Cin;
    // vectorized copy3to4: 32 RGB (96B) -> 32 RGBA (128B) via vdelta (4th byte junk; weight[3]=0 -> ok). L1.
    HVX_Vector ctrl = *(const HVX_Vector*)copy3to4_cntrl; int c=0;
    for(; c+32<=W; c+=32){ HVX_Vector v=*(HVX_UVec*)(irow+(long)c*3);
      *(HVX_UVec*)(crow+(long)(pad+c)*4) = Q6_V_vdelta_VV(v, ctrl); }
    for(; c<W; c++){ unsigned char* d=crow+(long)(pad+c)*4; d[0]=irow[c*3]; d[1]=irow[c*3+1]; d[2]=irow[c*3+2]; d[3]=0; }
  }
}
// Compute ONE output row (Wo px, 32ch NHWC) from its 3 tap rows cp4_3[ky] = cp4_3 + ky*WP_PIL*4. out_row = out + oy*Wo*Cout.
static void stem_pil_out_row(unsigned char* out_row, unsigned char* cp4_3, const int* wq, int* bias, int* recip,
    int Wo, int Cout, int k, int zsh){
  for(int ox0=0; ox0<Wo; ox0+=32){
    int npix = Wo-ox0<32 ? Wo-ox0 : 32;
    HVX_Vector iv[9];                                        // gather 9 tap vectors (32 stride-2 RGBA each) from cp4 (L1)
    for(int ky=0; ky<k; ky++){ unsigned char* crow = cp4_3 + (long)ky*WP_PIL*4;
      for(int kx=0; kx<k; kx++){ int s0 = 2*ox0 + kx;        // cp4 word start; 32 values at stride 2
        HVX_Vector vlo=*(HVX_UVec*)(crow+(long)s0*4), vhi=*(HVX_UVec*)(crow+(long)(s0+32)*4);
        iv[ky*k+kx] = Q6_V_lo_W(Q6_W_vdeal_VVR(vhi, vlo, -4)); } }   // word-deal -> even words = the stride-2 gather
    unsigned char* orow = out_row + (long)ox0*Cout;
    unsigned char g4s[8*128] __attribute__((aligned(128)));   // stage the 8 group vectors (1KB); reg staging spills -> rc=39
    // Level-1 pack (SHIFT-OR, 6 ops vs the old PBv+vshuff 11): requant_vec returns a clean u8 (word = 0x000000VV, upper
    // bytes 0), so ORing byte-shifted channels packs 4 channels into 1 word with NO collision: word px = [c0,c1,c2,c3].
    #define G4(w0_,x_,y_,z_) Q6_V_vor_VV(Q6_V_vor_VV((w0_), Q6_Vw_vasl_VwR((x_),8)), Q6_V_vor_VV(Q6_Vw_vasl_VwR((y_),16), Q6_Vw_vasl_VwR((z_),24)))
    for(int sg=0; sg<4; sg++){ int bb=8*sg;                  // 8 accumulators = 8-way ILP to hide the vrmpy-Rt latency
      HVX_Vector a0=Q6_V_vsplat_R(bias[bb+0]),a1=Q6_V_vsplat_R(bias[bb+1]),a2=Q6_V_vsplat_R(bias[bb+2]),a3=Q6_V_vsplat_R(bias[bb+3]),
                 a4=Q6_V_vsplat_R(bias[bb+4]),a5=Q6_V_vsplat_R(bias[bb+5]),a6=Q6_V_vsplat_R(bias[bb+6]),a7=Q6_V_vsplat_R(bias[bb+7]);
      for(int t=0;t<9;t++){ HVX_Vector x=iv[t]; const int* w=wq+t*32+bb;
        a0=Q6_Vw_vrmpyacc_VwVubRb(a0,x,w[0]); a1=Q6_Vw_vrmpyacc_VwVubRb(a1,x,w[1]);
        a2=Q6_Vw_vrmpyacc_VwVubRb(a2,x,w[2]); a3=Q6_Vw_vrmpyacc_VwVubRb(a3,x,w[3]);
        a4=Q6_Vw_vrmpyacc_VwVubRb(a4,x,w[4]); a5=Q6_Vw_vrmpyacc_VwVubRb(a5,x,w[5]);
        a6=Q6_Vw_vrmpyacc_VwVubRb(a6,x,w[6]); a7=Q6_Vw_vrmpyacc_VwVubRb(a7,x,w[7]); }
      a0=requant_vec(a0,Q6_V_vsplat_R(recip[bb+0]),zsh); a1=requant_vec(a1,Q6_V_vsplat_R(recip[bb+1]),zsh);   // clamp is load-bearing
      a2=requant_vec(a2,Q6_V_vsplat_R(recip[bb+2]),zsh); a3=requant_vec(a3,Q6_V_vsplat_R(recip[bb+3]),zsh);
      a4=requant_vec(a4,Q6_V_vsplat_R(recip[bb+4]),zsh); a5=requant_vec(a5,Q6_V_vsplat_R(recip[bb+5]),zsh);
      a6=requant_vec(a6,Q6_V_vsplat_R(recip[bb+6]),zsh); a7=requant_vec(a7,Q6_V_vsplat_R(recip[bb+7]),zsh);
      *(HVX_Vector*)(g4s+(long)(2*sg)*128)=G4(a0,a1,a2,a3); *(HVX_Vector*)(g4s+(long)(2*sg+1)*128)=G4(a4,a5,a6,a7);
    }
    #undef G4
    // level-2: 3-level word-transpose the 8 groups -> NHWC (4px x 32ch/vector). 12 vshuff + 8 vmem. R = -(byte granularity).
    HVX_Vector g0=*(HVX_Vector*)(g4s+0*128),g1=*(HVX_Vector*)(g4s+1*128),g2=*(HVX_Vector*)(g4s+2*128),g3=*(HVX_Vector*)(g4s+3*128),
               g5v=*(HVX_Vector*)(g4s+5*128),g4v=*(HVX_Vector*)(g4s+4*128),g6=*(HVX_Vector*)(g4s+6*128),g7=*(HVX_Vector*)(g4s+7*128);
    HVX_VectorPair p01=Q6_W_vshuff_VVR(g1,g0,-4),p23=Q6_W_vshuff_VVR(g3,g2,-4),p45=Q6_W_vshuff_VVR(g5v,g4v,-4),p67=Q6_W_vshuff_VVR(g7,g6,-4);
    HVX_VectorPair q0=Q6_W_vshuff_VVR(Q6_V_lo_W(p23),Q6_V_lo_W(p01),-8),q1=Q6_W_vshuff_VVR(Q6_V_hi_W(p23),Q6_V_hi_W(p01),-8),
                   q2=Q6_W_vshuff_VVR(Q6_V_lo_W(p67),Q6_V_lo_W(p45),-8),q3=Q6_W_vshuff_VVR(Q6_V_hi_W(p67),Q6_V_hi_W(p45),-8);
    HVX_VectorPair r0=Q6_W_vshuff_VVR(Q6_V_lo_W(q2),Q6_V_lo_W(q0),-16),r1=Q6_W_vshuff_VVR(Q6_V_hi_W(q2),Q6_V_hi_W(q0),-16),
                   r2=Q6_W_vshuff_VVR(Q6_V_lo_W(q3),Q6_V_lo_W(q1),-16),r3=Q6_W_vshuff_VVR(Q6_V_hi_W(q3),Q6_V_hi_W(q1),-16);
    if(npix==32){   // orow 128-aligned -> :nt aligned stores (output never re-read by the stem)
      vstnt(orow+ 0*Cout,Q6_V_lo_W(r0)); vstnt(orow+ 4*Cout,Q6_V_hi_W(r0));
      vstnt(orow+ 8*Cout,Q6_V_lo_W(r1)); vstnt(orow+12*Cout,Q6_V_hi_W(r1));
      vstnt(orow+16*Cout,Q6_V_lo_W(r2)); vstnt(orow+20*Cout,Q6_V_hi_W(r2));
      vstnt(orow+24*Cout,Q6_V_lo_W(r3)); vstnt(orow+28*Cout,Q6_V_hi_W(r3));
    } else { HVX_Vector rr[8]={Q6_V_lo_W(r0),Q6_V_hi_W(r0),Q6_V_lo_W(r1),Q6_V_hi_W(r1),Q6_V_lo_W(r2),Q6_V_hi_W(r2),Q6_V_lo_W(r3),Q6_V_hi_W(r3)};
      if((npix&3)==0){ for(int j=0; j*4<npix; j++) *(HVX_UVec*)(orow+(long)j*4*Cout)=rr[j]; }   // mult-4 partial: direct stores
      else { for(int j=0;j<8;j++) *(HVX_Vector*)(g4s+(long)j*128)=rr[j]; __builtin_memcpy(orow, g4s, (long)npix*Cout); } }
  }
}
static void stem_conv_pil(unsigned char* out, unsigned char* in, signed char* wi8, int* bias, int* recip,
   int Cin,int H,int W,int Cout,int k,int stride,int pad,int zsh,int Ho,int Wo,int oy0,int oy1, unsigned char* cp4, int xzp){
  const int* wq = (const int*)wi8;                          // wq[tap*32+ch] = the 4 i8 for (tap,ch) as an Rt scalar
  // cp4: caller-provided ARENA scratch (3*WP_PIL*4, per-worker slice) -- NOT the stack (stem_worker stack is near the
  // ~16KB FastRPC limit; any added stack array -> rc=39).
  for(int oy=oy0; oy<oy1; oy++){
    for(int ky=0; ky<k; ky++) stem_pil_fill(cp4+(long)ky*WP_PIL*4, in, oy*stride-pad+ky, H,W,Cin,pad,xzp);
    // prefetch the NEXT output row's 2 new input rows (2oy+2, 2oy+3) during this row's compute (no HW prefetcher).
    if(oy+1<oy1){ unsigned rb=(unsigned)(W*Cin)/128;
      for(int r=2;r<=3;r++){ int ii=oy*stride+r; if(ii>=0&&ii<H && rb) l2f(in+(long)ii*W*Cin,128,rb); } }
    stem_pil_out_row(out+(long)oy*Wo*Cout, cp4, wq, bias, recip, Wo, Cout, k, zsh);
  }
}
typedef struct { unsigned char *out,*in; int *wvec,*bias,*recip; int Cin,H,W,Cout,k,stride,pad,zsh,Ho,Wo,oy0,oy1,lock,vrmpy,xzp; unsigned char* cp4; } stemwork_t;
static void* stem_worker(void* arg){
  stemwork_t* w=arg;
  if(w->lock) qurt_hvx_lock(1);
  stem_conv_pil(w->out,w->in,(signed char*)w->wvec,w->bias,w->recip,w->Cin,w->H,w->W,w->Cout,w->k,w->stride,w->pad,w->zsh,w->Ho,w->Wo,w->oy0,w->oy1,w->cp4,w->xzp);
  if(w->lock) qurt_hvx_unlock();
  return 0;
}
// HVX head: OP_HEAD's per-element GAP work in INT32, vectorized over the CONTIGUOUS channel axis. The natural
// form (fold Mul(gate)+Add(res)+Relu into a per-channel float GAP) is HW*C SCALAR FLOAT, and HVX v65 has NO float
// vector unit, so it runs one-at-a-time on the scalar FP pipe (~24 cyc/elem) -- the whole ~3ms of OP_HEAD. Same two
// rules as setail_d32/gap_accum_d32: (1) get out of float -- fold Asc*gate[c] into a per-channel int32 multiplier;
// (2) it is already NHWC with c innermost, so the accumulator vectorizes over channels for free.
// Lane order: Q6_Wuh_vzxt_Vub -> Q6_Wuw_vzxt_Vuh puts the 128 channels in the SAME {0,2,1,3} permutation
// gap_accum_nhwc uses, so the multipliers are pre-permuted the same way and the readback un-permutes identically.
// The relu is exact in fixed point (2^S > 0 => max(0,v) commutes with the scaling). The numpy mirror _head (reference.py)
// is the bit-exact reference.
// `scratch` = the conv d32in buffer. head is the LAST op, so all conv scratch is dead: reuse it rather than growing
// the stack (same trick as se_gate's "reuse GAP accumulator"). This is NOT cosmetic -- head_hvx's mcp[]+acc[]+ssum[]
// on the stack took interp's frame from 9472 B to 21248 B (`allocframe(#0x2500)` -> `r29 = add(r29,#-0x5300)`), which
// blows the 16 KB FastRPC DSP thread stack: the DSP faults and the invoke returns EPERM. d32in is 282880 B here vs
// the 12288 B this needs at the worst case C=1536, and statics are not an option (they would make .bss NOBITS, and
// every LOAD segment in this .so is currently FileSiz==MemSiz -- the loader is never asked to zero anything).
// One 128-byte block of the head's gated-residual-relu GAP, accumulated into ap[0..3] (4 x 32 i32 lanes).
// A SECOND core, deliberately not ewise_blk: the head works in i32 end to end (u8 -> u16 -> u32 double widen, i32
// multiply, relu as a vmax, running sum over all HW with no per-block requant), which is why its S is bounded by
// the int32 accumulator (S<=30) rather than by an i16 product (S<=15). Folding it into the i16 chain core would
// change its numerics and risk overflow -- carving one kernel's shape into the "generic" core is the mistake this
// whole exercise is undoing, and it is no better in this direction. head_hvx and head_d32 DO share this exactly,
// so it lives here once; the callers pass the loaded vectors (one path loads unaligned, the other aligned) and
// pre-offset `mcv`/`ap`.
__attribute__((always_inline)) static inline void head_blk(HVX_Vector cv, HVX_Vector rv, const HVX_Vector* mcv,
    HVX_Vector* ap, HVX_Vector vzc, HVX_Vector vzr, HVX_VecW vmr, HVX_Vector vz, int hasres){
  HVX_VectorPair hh=Q6_Wuh_vzxt_Vub(cv);
  HVX_VectorPair l0=Q6_Wuw_vzxt_Vuh(Q6_V_lo_W(hh)), l1=Q6_Wuw_vzxt_Vuh(Q6_V_hi_W(hh));
  HVX_Vector x[4]; x[0]=Q6_V_lo_W(l0); x[1]=Q6_V_hi_W(l0); x[2]=Q6_V_lo_W(l1); x[3]=Q6_V_hi_W(l1);
  HVX_Vector r[4];
  if(hasres){ HVX_VectorPair hr=Q6_Wuh_vzxt_Vub(rv);
    HVX_VectorPair r0=Q6_Wuw_vzxt_Vuh(Q6_V_lo_W(hr)), r1=Q6_Wuw_vzxt_Vuh(Q6_V_hi_W(hr));
    r[0]=Q6_V_lo_W(r0); r[1]=Q6_V_hi_W(r0); r[2]=Q6_V_lo_W(r1); r[3]=Q6_V_hi_W(r1); }
  for(int j=0;j<4;j++){
    HVX_VecW v = ((HVX_VecW)Q6_Vw_vsub_VwVw(x[j],vzc)) * ((HVX_VecW)mcv[j]);
    if(hasres) v = v + ((HVX_VecW)Q6_Vw_vsub_VwVw(r[j],vzr)) * vmr;
    // vz is the clamp FLOOR, chosen by the caller: 0 for a head that fused a Relu (the gated/residual block's,
    // exact in fixed point), INT_MIN for one that did not -- max(v,INT_MIN) == v. Branchless, so a head with no
    // Relu costs nothing, and the loop keeps one instruction. Applying the clamp unconditionally was wrong on a
    // SIGNED input (zc != 0): it zeroed every negative. Invisible on models whose head input is post-relu. (#88)
    ap[j] = Q6_Vw_vadd_VwVw(ap[j], Q6_Vw_vmax_VwVw((HVX_Vector)v, vz));
  }
}
// Head gemm over an output range: emb[o] = gws[o]*(s_bn*(dot(qu,gw[o]) - z_bn*gwsum[o])) + gb[o]. gw (O x C, e.g.
// 1.28MB) is >L2 -> a sliding l2fetch window makes each thread's weight stream BW-bound (not DDR-latency-bound).
static void head_gemm_range(float* emb, unsigned char* qu, signed char* gw, float* gws, float* gb, float* gwsum,
   int C, float z_bn, float s_bn, int o0, int o1){
#ifndef HEAD_PF_AHEAD
#define HEAD_PF_AHEAD 4
#endif
  int Cp=(C+127)&~127, gpf=Cp/128;   // PADDED row stride: dot_u8i8's weight loads are aligned vmem, so a row
  for(int p=o0; p<o0+HEAD_PF_AHEAD && p<o1; p++) l2f(gw+(long)p*Cp, 128, gpf);   // stride of C would fault at C%128
  for(int o=o0; o<o1; o++){
    if(o+HEAD_PF_AHEAD<o1) l2f(gw+(long)(o+HEAD_PF_AHEAD)*Cp, 128, gpf);
    float a = s_bn*((float)dot_u8i8(qu, gw+(long)o*Cp, C) - z_bn*gwsum[o]);
    emb[o] = gws[o]*a + gb[o]; }
}
typedef struct { float* emb; unsigned char* qu; signed char* gw; float *gws,*gb,*gwsum; int C; float z_bn,s_bn; int o0,o1; } headgemm_t;
static void* head_gemm_worker(void* a){ headgemm_t* w=a;
  head_gemm_range(w->emb,w->qu,w->gw,w->gws,w->gb,w->gwsum,w->C,w->z_bn,w->s_bn,w->o0,w->o1); return 0; }
// ---- THE HEAD, in two layouts. What genuinely differs between them is only how the gated residual-relu GAP is
// ACCUMULATED; the fixed-point scale choice before it and the BN -> quant -> Gemm after it are IDENTICAL, and were
// written twice. Both accumulators now produce accd[C] in NATURAL channel order, which is what lets the epilogue be
// shared at all -- head_hvx used to carry the {0,2,1,3} vzxt un-permutation into its own BN loop.
// ★ This also fixed a latent inconsistency rather than just deduplicating: head_d32 called head_gemm_range
// DIRECTLY, so a d32 head never got the size-gated 2-way gemm threading head_hvx has had since it measured
// -17.4% on MobileNetV2's 1.28MB weight matrix. distill (O=512 x C=512 = 262KB) is under the gate either way, so
// nothing moves today -- but a d32 head with a big classifier would silently have run single-threaded.
typedef struct { float Asc,Asr,inv_sbn,z_bn,s_bn,inv2S; float *P,*Qb,*gws,*gb,*gwsum; int S,mr; } headp_t;
static headp_t head_prep(float* blob, int C, int O, int HW){
  headp_t h; h.Asc=blob[0]; h.Asr=blob[1]; h.inv_sbn=blob[2]; h.z_bn=blob[3]; h.s_bn=blob[4];
  h.P=blob+5; h.Qb=h.P+C; h.gws=h.Qb+C; h.gb=h.gws+O; h.gwsum=h.gb+O;
  // Pick S: the per-channel multiplier needs headroom, and the HW-long int32 accumulator must not overflow.
  // (setail_d32 targets int16 and does NOT accumulate, so its S rule is not safe here.)
  float mx = h.Asc>h.Asr?h.Asc:h.Asr; if(mx<=0.0f) mx=1e-6f;
  h.S=30;
  while(h.S>0 && mx*(float)(1<<h.S) >= 1.0e6f) h.S--;                              // multiplier headroom
  while(h.S>0 && (float)HW*255.0f*(h.Asc+h.Asr)*(float)(1<<h.S) >= 2.0e9f) h.S--;  // int32 accumulator bound
  h.inv2S = 1.0f/(float)(1<<h.S); h.mr = roundi(h.Asr*(float)(1<<h.S));
  return h;
}
// natural-order GAP sums -> BN -> quant -> Gemm. BN+quant stays per-channel float: it is C-long, not HW*C.
// `gap[c]` arrives ALREADY un-scaled (each accumulator applies its own inv2S, or is float to begin with). That is
// deliberate: the NHWC tail sums in float and the vector bodies sum in int32 fixed point, so routing everything
// through an int array would put a roundi()/inv2S round-trip on the float path and stop it being bit-identical.
static void head_tail(float* emb, const float* gap, const headp_t* h, signed char* gw, int C, int O, pool_t* pool,
    unsigned char* scratch){
  unsigned char* qu = SCR_TAKE(scratch, C);   // was qu[1536]: a fixed bound on the head width with nothing checking it
  for(int c=0;c<C;c++){
    float bn=gap[c]*h->P[c]+h->Qb[c];
    int q=roundi(bn*h->inv_sbn)+(int)h->z_bn; if(q<0)q=0; if(q>255)q=255; qu[c]=(unsigned char)q; }
  // Gemm: BW-bound streaming the >L2 gw (MNv2: O=1000 x C=1280 = 1.28MB). SIZE-GATED on the measured usable L2
  // (~384KB, pfcal.py) -- threading helps only when the weight matrix does not fit and the op really streams DDR:
  //   MNv2    O=1000 x C=1280 = 1.28MB > L2 -> 443 -> 366 us (-17.4%), whole model -1.8%
  //   distill O=512  x C=512  = 262KB  < L2 -> 184 -> 192 us (+4%), pool overhead with no stream to split
#ifndef HEAD_GEMM_TH_BYTES
#define HEAD_GEMM_TH_BYTES (384*1024)
#endif
  if((long)O*C >= HEAD_GEMM_TH_BYTES && pool && pool->up && O >= 64){
    headgemm_t w0={emb,qu,gw,h->gws,h->gb,h->gwsum,C,h->z_bn,h->s_bn,0,O/2},
               w1={emb,qu,gw,h->gws,h->gb,h->gwsum,C,h->z_bn,h->s_bn,O/2,O};
    pool_run2(pool, head_gemm_worker, &w0, &w1);
  } else head_gemm_range(emb, qu, gw, h->gws, h->gb, h->gwsum, C, h->z_bn, h->s_bn, 0, O);
}
// NHWC [HW][C], c innermost: the accumulator vectorizes over channels for free, 4 int32 vectors per 128-ch block.
static void head_gap_nhwc(float* gap, unsigned char* conv, unsigned char* res, float* gate, const headp_t* h,
    int C,int HW,int zc,int zr,int hasres,int has_gate,int relu, unsigned char* scratch){
  int nblk=C/128, Cv=nblk*128;
  int* mcp=SCR_TAKE(scratch, C*4); int* acc=SCR_TAKE(scratch, C*4);
  for(int c=0;c<Cv;c++){ int l=c&127,rr=l&3,gg=(rr==1?2:(rr==2?1:rr));   // multipliers pre-permuted to acc order
    mcp[(c&~127)+(gg<<5)+(l>>2)] = roundi(h->Asc*(has_gate?gate[c]:1.0f)*(float)(1<<h->S)); }
  HVX_Vector* ap=(HVX_Vector*)acc; HVX_Vector* mcv=(HVX_Vector*)mcp;
  for(int v=0;v<nblk*4;v++) ap[v]=Q6_V_vzero();
  HVX_Vector vzc=Q6_V_vsplat_R(zc), vzr=Q6_V_vsplat_R(zr);
  HVX_Vector vz=relu?Q6_V_vzero():Q6_V_vsplat_R(0x80000000);   // clamp floor: 0 = fused relu, INT_MIN = none
  HVX_VecW vmr=(HVX_VecW)Q6_V_vsplat_R(h->mr);
  for(int i=0;i<HW;i++){
    HVX_UVec* ic=(HVX_UVec*)(conv+(long)i*C);              // UNALIGNED: C need not be mult-128
    HVX_UVec* ir=hasres?(HVX_UVec*)(res+(long)i*C):ic;
    for(int blk=0;blk<nblk;blk++)
      head_blk(ic[blk], hasres?ir[blk]:ic[blk], mcv+blk*4, ap+blk*4, vzc,vzr,vmr,vz, hasres);
  }
  for(int c=0;c<Cv;c++){ int l=c&127,rr=l&3,gg=(rr==1?2:(rr==2?1:rr));   // {0,2,1,3} -> natural channel order
    gap[c]=(float)acc[(c&~127)+(gg<<5)+(l>>2)]*h->inv2S; }
  // Scalar tail for C not a multiple of 128. It accumulates in FLOAT while the vector body above accumulates in
  // int32 fixed point, so the leftover channels round differently from every other channel -- which looks like an
  // inconsistency to fix, and is not.
  // ★ MEASURED 2026-07-26, by converting it to the same fixed point and being caught by the gate. It is ~30%
  // FASTER (the probes, whose C=64 puts EVERY channel here, went 1.24 -> 0.835 ms: v65 has no vector float unit,
  // so this loop runs ~24 cyc/elem and was about half of each probe's runtime) -- and it costs cosine: dw_k3_s1
  // and dw_k7_s1 fell 1.0000000 -> 0.9999994/0.9999992. FLOAT IS CLOSER TO THE ORACLE, which is float ONNX.
  // Declined, because the trade is backwards: the speedup lands only on C%128 != 0, which NEITHER real model has
  // (distill C=512, MNv2 C=1280), while the cost lands on the probes' exact 1.0000000 -- the property that makes
  // them able to validate a kernel with no recorded baseline. Revisit only for a real model with a small head.
  const float vlof = relu ? 0.0f : -3.0e38f;   // hoisted: the flag never enters the inner loop
  for(int c=Cv;c<C;c++){
    float ag=h->Asc*(has_gate?gate[c]:1.0f), s=0.0f;   // hoisting Asc*gate[c] keeps the same association -> exact
    for(int i=0;i<HW;i++){ float v=ag*((float)conv[(long)i*C+c]-zc);
      if(hasres) v+=h->Asr*((float)res[(long)i*C+c]-zr); if(v<vlof)v=vlof; s+=v; }
    gap[c]=s;   // already in real units -- no fixed-point round-trip, so this stays bit-identical
  }
}
// d32 [Ho][C/32][Wop][32]: a 128-byte block is 4 pixels x 32 channels, so the accumulator's lanes are (pixel,
// channel) PAIRS and the 4 pixel-slots must fold into 32 channel sums per chunk. THIS is why the two GAPs cannot
// share one addressing loop the way the elementwise kernels do -- the lane MEANING differs, not just the stride.
static void head_gap_d32(float* restrict gap, unsigned char* restrict conv, unsigned char* restrict res, float* gate,
    const headp_t* h, int C,int Ho,int Wo,int Wop,int zc,int zr,int hasres,int has_gate,int relu){
  int nchunk=C/32; long rowstride=(long)nchunk*Wop*32; int wg=Wo>>2, wtail=Wo&3;
  HVX_Vector vzc=Q6_V_vsplat_R(zc), vzr=Q6_V_vsplat_R(zr);
  HVX_Vector vz=relu?Q6_V_vzero():Q6_V_vsplat_R(0x80000000);   // clamp floor: 0 = fused relu, INT_MIN = none
  HVX_VecW vmr=(HVX_VecW)Q6_V_vsplat_R(h->mr);
  int a128[128] __attribute__((aligned(128))), mcp[128] __attribute__((aligned(128))), mcn[32];
  { long span=(long)Ho*rowstride; unsigned nb=(unsigned)(span/128); if(nb>65535) nb=65535;   // whole-slice prefetch
    if(nb){ l2f(conv,128,nb); if(hasres) l2f(res,128,nb); } }
  for(int ch=0; ch<nchunk; ch++){
    for(int c=0;c<32;c++) mcn[c]=roundi(h->Asc*(has_gate?gate[ch*32+c]:1.0f)*(float)(1<<h->S)); // natural (for tail)
    for(int b=0;b<128;b++){ int rr=b&3, gg=(rr==1?2:(rr==2?1:rr));   // mcp in the vzxt {0,2,1,3} acc-lane order
      mcp[(gg<<5)+(b>>2)] = mcn[b&31]; }
    HVX_Vector* mcv=(HVX_Vector*)mcp; HVX_Vector* ap=(HVX_Vector*)a128;
    for(int v=0;v<4;v++) ap[v]=Q6_V_vzero();
    for(int hh=0; hh<Ho; hh++){
      HVX_Vector* ic=(HVX_Vector*)(conv+(long)hh*rowstride+(long)ch*Wop*32);
      HVX_Vector* ir=hasres?(HVX_Vector*)(res+(long)hh*rowstride+(long)ch*Wop*32):ic;
      for(int g=0; g<wg; g++) head_blk(ic[g], hasres?ir[g]:ic[g], mcv, ap, vzc,vzr,vmr,vz, hasres);
    }
    int csum[32];
    for(int c=0;c<32;c++){   // reduce the 4 pixel-slots -> natural-order channel sum
      int sv=0;
      for(int slot=0;slot<4;slot++){ int b=slot*32+c, rr=b&3, gg=(rr==1?2:(rr==2?1:rr)); sv+=a128[(gg<<5)+(b>>2)]; }
      csum[c]=sv;
    }
    if(wtail){ unsigned char* pc0=conv+(long)ch*Wop*32; unsigned char* pr0=hasres?res+(long)ch*Wop*32:pc0;
      for(int c=0;c<32;c++){ int sv=csum[c], m=mcn[c];            // reuse precomputed mcp (no per-elem roundi)
        for(int hh=0;hh<Ho;hh++){ unsigned char* pc=pc0+(long)hh*rowstride; unsigned char* pr=pr0+(long)hh*rowstride;
          for(int w=wg*4;w<Wo;w++){ int vv=((int)pc[w*32+c]-zc)*m;
            if(hasres) vv+=((int)pr[w*32+c]-zr)*h->mr; if(vv<0)vv=0; sv+=vv; } }
        csum[c]=sv; } }
    for(int c=0;c<32;c++) gap[ch*32+c]=(float)csum[c]*h->inv2S;
  }
}

// HVX asm (nnlib dwconv2dbbb_s1_3x3). Signature DECODED from the asm's stack-arg loads (memw(sp+#N<<2), first stack
// arg = word16 = arg6), NOT the header C-model's arg names (which differ: the asm drops filt_height and treats
// recip_level as a POINTER). Real 17-arg contract:
//   (in, filt, out, next_in_width, next_out_width, next_in_width_32, next_out_width_32,
//    depth[=C, asm does /32], out_width, out_height, filt_width, filt_zero,
//    bias_sum*, max*(int[64],128-al), recip_level*(int[32] splat,128-al, read via vmem), recip_shift, stride_height)
// All vector operands (in/out/filt/bias/max/recip) use ALIGNED vmem -> every buffer must be 128-byte aligned;
// next_in_width/next_out_width (Wp*32/owp*32) must be multiples of 128 (Wp,owp mult of 4). Loops out_height internally.
extern void dwconv2dbbb_s1_3x3_asm(unsigned char* in, unsigned char* filt, unsigned char* out,
   int next_in_width, int next_out_width, int next_in_width_32, int next_out_width_32,
   int depth, int out_width, int out_height, int filt_width, int filt_zero,
   int* bias_sum, int* max, int* recip_level, int recip_shift, int stride_height);
// 5xN (nnlib dwconv2dbbb_s1_5xN): filt_width HARDCODED 5, filt_height variable (arg11). IDENTICAL 17-arg contract to
// s1_3x3 (C prototype verified) -- no sbuf. For a 5x5 dw pass filt_height=5, stride_height=1. (s2_5xN deferred: its
// padding convention differs from s2_3x3 -- no in_left_skip arg -- so it needs its own dwconv_op setup.)
extern void dwconv2dbbb_s1_5xN_asm(unsigned char* in, unsigned char* filt, unsigned char* out,
   int next_in_width, int next_out_width, int next_in_width_32, int next_out_width_32,
   int depth, int out_width, int out_height, int filt_height, int filt_zero,
   int* bias_sum, int* max, int* recip_level, int recip_shift, int stride_height);
// 7xN (variable filter HEIGHT via filt_width=filt_size=kh; horizontal fixed at <=7 taps): same contract as 3x3 PLUS
// a trailing sbuf scratch (arg17, ~2KB, resets per column) and uses UNALIGNED vmemu for in/out (no in/out align req).
extern void dwconv2dbbb_s1_7xN_asm(unsigned char* in, unsigned char* filt, unsigned char* out,
   int next_in_width, int next_out_width, int next_in_width_32, int next_out_width_32,
   int depth, int out_width, int out_height, int filt_width, int filt_zero,
   int* bias_sum, int* max, int* recip_level, int recip_shift, int stride_height, void* sbuf);
// s2_3x3 (STRIDE-2 3x3 downsample dw): same 17-arg s1 contract PLUS an unused filler (arg17) and in_left_skip (arg18,
// =0 for SAME pad). stride_height=2; horizontal stride hardcoded to 2. Reads full-res d32in with stride, writes H/2 x W/2.
extern void dwconv2dbbb_s2_3x3_asm(unsigned char* in, unsigned char* filt, unsigned char* out,
   int next_in_width, int next_out_width, int next_in_width_32, int next_out_width_32,
   int depth, int out_width, int out_height, int filt_width, int filt_zero,
   int* bias_sum, int* max, int* recip_level, int recip_shift, int stride_height, int filler, int in_left_skip);
// s2_5xN (STRIDE-2 5x5, EfficientNet/MobileNetV3 downsample): the s1_5xN contract VERBATIM with stride_height=2 --
// verified by arg-slot decode (it reads sp+#16..#26 = args 6..16 and NOTHING else). Unlike s2_3x3 it takes NO
// in_left_skip (arg18), so the caller must land the phase exactly; padL = 4*s+pad does that for every k (it makes
// in_left_skip = padL-(oLp*s+pad) = 0 identically), and k=3's long-standing padL=9 is just that formula's k=3 case.
extern void dwconv2dbbb_s2_5xN_asm(unsigned char* in, unsigned char* filt, unsigned char* out,
   int next_in_width, int next_out_width, int next_in_width_32, int next_out_width_32,
   int depth, int out_width, int out_height, int filt_height, int filt_zero,
   int* bias_sum, int* max, int* recip_level, int recip_shift, int stride_height);
#ifndef DW_TH
#define DW_TH 4       // lever-1 height-tile size (output rows/asm call); prefetch next tile's d32in during compute.
#endif                // TH=4 swept best: -35% on mem-bound tall shapes (d32in>L2), neutral where d32in already fits L2.
// Kernel choice is made HOST-side (registry.pick_dw) and arrives as op[DW_kern]; there is no fallback --
// an unsupported shape fails the BUILD. The input is ALWAYS d32 (asserted at emit): either the T1a bordered
// buffer its producer wrote, or a border-fill + contiguous d32 repad off the producer's d32. Output goes back
// through from_d32_asm's HVX transpose when a consumer wants NHWC. Both compute paths use the SAME d32 layout so
// they share the repad: input [Hp][D][Wp][32], output [H][D][owp][32].
// dw is overhead-bound (1-7 GMAC/s, depthwise=1 MAC/tap; 1.5MB traffic=0.15ms@10GB/s vs 2.2ms actual) -> THREADS.
// Splits the DW_TH output-row tiles across HVX units; shared read-only d32in/filt/recipv, per-thread mm (own min/max).
typedef struct { unsigned char *d32in,*d32out,*out,*filt; int *bias,*recipv,*mm;
  int niw,niw32,now,now32,C,oW,owt,oH,kw,kh,fz,rsh,s,ils,oLp,D,outd,realC,kern,tile; void* sbuf; int oy0,oy1,lock;
  unsigned long long* prof; } dwwork_t;
static void* dw_worker(void* arg){
  dwwork_t* w=arg;
  if(w->oy0>=w->oy1) return 0;                    // empty slice (pool worker handed no rows) -> don't touch mm/sbuf
  int* mm=w->mm; int* recipv=w->recipv;
  if(w->lock) qurt_hvx_lock(1);
  for(int i=0;i<32;i++){ mm[i]=-0x7fffffff; mm[32+i]=0x7fffffff; }   // this thread's own min/max accumulator
  for(int oy0=w->oy0; oy0<w->oy1; oy0+=w->tile){
    int nh=(w->oy1-oy0<w->tile)?(w->oy1-oy0):w->tile, oyn=oy0+nh;
    if(oyn<w->oH){ int nhn=(w->oH-oyn<DW_TH)?(w->oH-oyn):DW_TH;
      l2f(w->d32in+(long)oyn*w->s*w->niw, 128, (unsigned)((long)(nhn*w->s+w->kh)*w->niw/128)); }
    unsigned char* ti=w->d32in+(long)oy0*w->s*w->niw; unsigned char* to=w->d32out+(long)oy0*w->now;
    // out_width = owt (the oLp-PADDED width) whenever the output has a left pad: every kernel here writes from
    // column 0, and the consumer skips oLp -- so the kernel must produce oLp extra columns or the last oLp real
    // columns are never written (measured: 7xN with oW under residency = 0.9999552, with owt = exact).
    int ow_k = w->oLp ? w->owt : w->oW;   // kernel id chosen host-side by registry.pick_dw; see registry.py
    switch(w->kern){
      case 1: dwconv2dbbb_s1_3x3_asm(ti,w->filt,to,w->niw,w->now,w->niw32,w->now32,w->C,ow_k,nh,w->kw,w->fz,w->bias,mm,recipv,w->rsh,1); break;
      case 2: dwconv2dbbb_s1_5xN_asm(ti,w->filt,to,w->niw,w->now,w->niw32,w->now32,w->C,ow_k,nh,w->kh,w->fz,w->bias,mm,recipv,w->rsh,1); break;
      case 3: dwconv2dbbb_s1_7xN_asm(ti,w->filt,to,w->niw,w->now,w->niw32,w->now32,w->C,ow_k,nh,w->kh,w->fz,w->bias,mm,recipv,w->rsh,1,w->sbuf); break;
      case 4: dwconv2dbbb_s2_3x3_asm(ti,w->filt,to,w->niw,w->now,w->niw32,w->now32,w->C,w->owt,nh,w->kw,w->fz,w->bias,mm,recipv,w->rsh,2,0,w->ils); break;
      case 5: dwconv2dbbb_s2_5xN_asm(ti,w->filt,to,w->niw,w->now,w->niw32,w->now32,w->C,ow_k,nh,w->kh,w->fz,w->bias,mm,recipv,w->rsh,2); break;
    }
    unsigned long long* prof=w->prof;
    // rc is mult-32 (_pad32) and D>=2 is asserted at emit for an NHWC-output dw, so this is the only unpack.
    PHASE(PH_DW_UNPACK) if(!w->outd){ unsigned char* tv=to+(long)w->oLp*32; int rc=w->realC;
      from_d32_asm(tv, w->owt*32, w->out+(long)oy0*w->oW*rc, w->oW, nh, rc); }
  }
  if(w->lock) qurt_hvx_unlock();
  return 0;
}
static void dwconv_op(unsigned char* arena, unsigned char* filt, int* bias, int* minmax, int* op, pool_t* pool,
   unsigned long long* prof){
  (void)minmax;
  int ind=op[DW_ind], outd=op[DW_outd], ind_bordered=op[DW_ind_bord];        // d32-residency: input already d32 (repad) / write d32 out
                                                           // ind_bordered: producer wrote the interior -> border-zap only (T1a)
  unsigned char* out=arena+op[DW_out]; unsigned char* in=arena+op[DW_src];
  unsigned char* d32in=arena+op[DW_d32in]; unsigned char* d32out=outd?(arena+op[DW_out]):(arena+op[DW_d32out]);
  int C=op[DW_C],H=op[DW_H],W=op[DW_W],kh=op[DW_kh],kw=op[DW_kw],fz=op[DW_fz],recip=op[DW_recip],rsh=op[DW_rsh];
  int s=op[DW_s]?op[DW_s]:1;                                    // stride (1 or 2); s==2 -> s2_3x3 downsample
  // GEOMETRY comes from the record (op[24..34]), derived once in geometry.dw_geom -- see geometry.py's header for
  // why. This kernel may only do ARITHMETIC on those values (D, niw, now, aux below), never re-derive a policy.
  // Verified on-device under -DGEOM_ASSERT (transitional, since removed): every field equalled the old C
  // derivation for k=3/5/7 x s=1/2 and all 17 of MobileNetV2's depthwise ops.
  int Cp=op[DW_Cp], pad=op[DW_pad], ofw=op[DW_ofw], padL=op[DW_padL], Wp=op[DW_Wp], Hp=op[DW_Hp];
  // Border fill value: conv padding is zero DEQUANTIZED, i.e. u8 == the activation zero point. See lower.py.
  int xzp=op[DW_xzp];
  int oH=op[DW_oH], oW=op[DW_oW], oLp=op[DW_oLp], ils=op[DW_ils], owt=op[DW_owt];
  int D=Cp/32;
  // EXACT nnlib-driver recipe for s2 (op_dwise_snode_8bit.c): the s2 vrmpy needs LEFT pad = 4 (in_left_pad). Its first
  // group then emits out_left_pad = (in_left_pad-required_w_before)/stride = (4-1)/2 = 1 JUNK output col which we DISCARD
  // (read output from col out_left_pad). in_left_skip = in_left_pad-(out_left_pad*s+required_w_before) = 4-(2+1) = 1 ->
  // asm arg (in_left_skip&1)*8 = 8. (padL 1 or 2 gave the asm too little context -> 2 junk cols not fixable by skip.)
  // left pad chosen so out_left_pad (junk cols discarded) is a MULTIPLE OF 4 -> the from_d32 read pointer (to+oLp*32)
  // stays 128-aligned (from_d32_asm uses aligned vmem). oLp=(padL-pad)/s; oLp=4 needs padL=9. arg=(in_left_skip&1)*8.
  // op[20]=in_left_pad override: RESIDENT dw uses in_left_pad=4 (ALIGNED, 4*32=128) so the producer can write the valid
  // region with an aligned store AND the dw writes d32-out (out_left_pad=1) for the consumer's repstream/valign to absorb
  // (no from_d32 -> which is why padL=9's oLp=4 was needed). oLp/ils/owt below derive from padL, so padL=4 -> oLp=1, ils=8.
  PHASE(PH_DW_REPAD)
  if(ind_bordered){                                   // T1a: the producer conv ALREADY wrote the full bordered
    long rowsz=(long)D*Wp*32;                                 // [Hp][D][Wp][32] interior into d32in (=op[5]). Only fill the
    d32_fill(d32in, xzp, (long)pad*rowsz);                    // borders (top/bottom pad rows + per-row left/right cols, incl
    d32_fill(d32in+(long)(pad+H)*rowsz, xzp, (long)(Hp-pad-H)*rowsz);   // the asm's ru(Wo,8) right-of-W junk). No interior copy.
    // ★★★ NOT A LEVER, and the phase timer says otherwise -- ABLATE BEFORE BELIEVING ONE. This block reads
    // 0.588ms on MobileNetV2, but removing it ENTIRELY makes the model 1.97% SLOWER (paired, 0/5 pairs), because
    // the zeroing also WARMS the bordered buffer into L2 just before the depthwise streams it. Two attempts to
    // make it cheaper were both null: an 8-byte tail in d32_zero (+0.9%) and `:nt` stores to skip write-allocate
    // (0.00% / +0.11%). A phase timer says where time is SPENT, not what is REMOVABLE.
    for(int h=0;h<H;h++){ unsigned char* row=d32in+(long)(h+pad)*rowsz;
      for(int d=0;d<D;d++){ unsigned char* ds=row+(long)d*(Wp*32);
        d32_fill(ds, xzp, (long)padL*32);
        d32_fill(ds+(long)(padL+W)*32, xzp, (long)(Wp-padL-W)*32); } }
  } else if(ind){                                            // d32-in: REPAD producer d32 [H][D][iwp][32] -> [Hp][D][Wp][32]
    int iwp2=op[DW_src_wop]?op[DW_src_wop]:((W+3)&(~3)); long rowsz=(long)D*Wp*32;  // producer's d32 row width (its Wop). BORDERS-only
    d32_fill(d32in, xzp, (long)pad*rowsz);                   // no transpose, reads producer d32 not NHWC). memd copies
    d32_fill(d32in+(long)(pad+H)*rowsz, xzp, (long)(Hp-pad-H)*rowsz);   // (d32_copy/d32_fill) not byte memcpy: op4 1.9->~0.2ms.
    for(int h=0;h<H;h++){ unsigned char* row=d32in+(long)(h+pad)*rowsz;
      for(int d=0;d<D;d++){ unsigned char* ds=row+(long)d*(Wp*32);
        d32_fill(ds, xzp, (long)padL*32);
        d32_copy(ds+(long)padL*32, in+(long)h*(D*iwp2*32)+(long)d*(iwp2*32), (long)W*32);
        d32_fill(ds+(long)(padL+W)*32, xzp, (long)(Wp-padL-W)*32); } }
  }   // NHWC input: impossible by construction, and asserted at emit -- see the pack() note at the top of the file
  int niw=D*Wp*32, niw32=Wp*32, now=D*owt*32, now32=owt*32; // input [Hp][D][Wp][32], output [oH][D][owt][32]
  { long fb=(long)D*kh*ofw*32; if(fb>=128) l2f(filt,128,(unsigned)(fb/128)); }   // weights are tiny -> prefetch once
  // 128-aligned aux buffers from the ION scratch (op[15], AL-aligned) -- NOT the stack (Hexagon frame is only
  // 8-aligned under -ffreestanding, so aligned(128) locals aren't guaranteed -> aligned vmem would fault).
  // s1_5xN/s1_7xN are nnlib's PER-CHANNEL-SCALE dw variants: they read the requant multiplier as
  // `vmem(recip_level++#1)` -- ONE 128B vector per 32-channel depth chunk -- whereas s1_3x3/s2_3x3 read a single
  // `vmem(recip_level)`. (nnlib's own driver mirrors this: it picks s1_3x3 only when !has_channel_scale.) We
  // quantize per tensor, so replicate the same recip across all D chunks. With a single vector, depth chunk 0 was
  // correct and chunks 1+ read off the end into mm (0x7fffffff) -> requant clipped to 0/255. Measured on a 5x5 dw
  // output: cos 0.268, chunk 0 exact, chunks 1-3 destroyed.
  int aux = D*128;                           // [0, D*128) = the per-chunk recip vectors; mm and sbuf follow
  int* recipv = (int*)(arena+op[DW_aux]);
  int* mm     = (int*)(arena+op[DW_aux]+aux);    // asm min/max buffer: max[0..31], min[32..63]
  for(int i=0;i<32*D;i++) recipv[i]=recip;
  for(int i=0;i<32;i++){ mm[i]=-0x7fffffff; mm[32+i]=0x7fffffff; }
  {
    // LEVER 1: height-TILE the asm call (the asm takes out_height + in/out pointers) and prefetch the NEXT tile's
    // d32in rows while THIS tile computes -- overlapped l2f, mirroring conv_op's chunk-ahead weight prefetch.
    // Loop over OUTPUT rows: output row oy0 reads INPUT row oy0*s (asm strides internally). Each tile self-contained.
    int tile=op[DW_tile]?op[DW_tile]:DW_TH, threads=op[DW_threads]?op[DW_threads]:1;   // SCHEDULE from the record (geometry.dw_sched)
    void* sbuf = arena+op[DW_aux]+aux+256;        // 7xN scratch (~2KB, resets per column); stays 128-aligned
    int tile0 = oH<tile?oH:tile;            // warm the first tile (pack wrote d32in top->bottom, so row 0 is coldest)
    l2f(d32in, 128, (unsigned)((long)(tile0*s+kh)*niw/128));
    // THREAD the tile loop across HVX units (dw is overhead-bound). Shared read-only d32in/filt/recipv; per-thread mm.
    dwwork_t base = { d32in,d32out,out,filt,bias,recipv,mm, niw,niw32,now,now32,Cp,oW,owt,oH,kw,kh,fz,rsh,s,ils,oLp,D,outd,C,op[DW_kern],tile, sbuf, 0,oH,0, prof };
    // dws are MEMORY-bound (2 threads share one memory port -> no gain; measured 16.40->16.86 threaded), so run DIRECT
    // by default (no barrier overhead). -DDW_THREAD routes through the pool (only useful if a dw is compute-bound).
    // SIZE-GATED 2-thread dw: threading gives ~1.9x DDR BW but adds barrier overhead. Measured (prof_ops): the big
    // hi-res s1 dw (op1 C=32 112x112: 0.362->0.322) WINS; smaller/s2 dws (56x56, s2) LOSE (op7 +0.05, op4 +0.08) --
    // barrier cost exceeds the memory gain below high res. So thread ONLY large s1 3x3 dws. d1.mm reuses the (kh==3-
    // unused) sbuf region at op[15]+384. The gate itself lives in lower.dw_sched (env DW_THREAD_MIN), not here.
    PHASE(PH_DW_TILE) if(threads==2){
      int tiles=(oH+tile-1)/tile, rt=((tiles+1)/2)*tile;
      dwwork_t d0=base, d1=base;
      d0.oy0=0;  d0.oy1=rt<oH?rt:oH; d0.mm=(int*)(arena+op[DW_aux]+aux);
      d1.oy0=rt<oH?rt:oH; d1.oy1=oH; d1.mm=(int*)(arena+op[DW_aux]+aux+256);
      pool_run2(pool, dw_worker, &d0, &d1);
    } else dw_worker(&base);   // base.mm = arena+op[15]+aux (set in the struct init)
  }
}

__attribute__((noinline)) void interp(unsigned char* arena, unsigned char* wts, int* ops, int nops,
   unsigned char* d32in, unsigned char* d32out, int* minmax, unsigned long long* prof){
  pool_t pool; pool.up=0;
  // The persistent qurt worker holds HVX unit 1 for the WHOLE run, so start it for any graph that has work to give
  // it -- every conv and every depthwise goes through pool_run2. The pool used to be started only for graphs
  // WITH a depthwise, on the theory that an idle worker hogging a unit would hurt a conv-only net; measured, the
  // opposite held (distill's 56 convs then ran one-context, 0 of them anywhere near the 202.5 GMAC/s single-context
  // cap), and a barrier at ~1us beats the ~0.1ms per-op pthread spawn it replaced.
  int has_work=0;
  for(int i=0;i<nops;i++){ int o=ops[i*INTS_PER_OP]; if(o==OP_DWCONV||o==OP_CONV){ has_work=1; break; } }
  if(has_work) pool_start(&pool);
  for(int i=0;i<nops;i++){
    int* op = ops + i*INTS_PER_OP;
    unsigned long long t0=HAP_perf_get_time_us();
    // CROSS-OP STATIC WEIGHT PREFETCH (Phase 1): warm the NEXT op's weights into L2 during THIS op's compute (weights are
    // read-only + statically located at op[3] for conv/dw/inconv). Capped at CROSS_PF_KB so it doesn't evict this op's
    // working set. Generic: hides the weight-DDR latency that weight-heavy ops expose at their start. 0 = off.
#ifndef CROSS_PF_KB
#define CROSS_PF_KB 64   // swept: 64KB = sweet spot (−0.08ms; classifier 0.253→0.234). 128 REGRESSES (evicts this op's set)
#endif
#if CROSS_PF_KB>0
    if(i+1<nops){ int* nx=ops+(i+1)*INTS_PER_OP; int o=nx[0];
      if(o==OP_CONV||o==OP_DWCONV||o==OP_INCONV){ L2FP(prof, wts+nx[3], 128, (CROSS_PF_KB*1024u)/128); } }
#endif
    if(op[0]==OP_CONV){
      conv_op(arena+op[CV_src], arena+op[CV_out], d32in, d32out, wts+op[CV_wt],
              (int*)(wts+op[CV_bias]), (int*)(wts+op[CV_recip]), minmax, op+CV_P0, prof, arena, &pool);
    } else if(op[0]==OP_SE_GATE){
      se_gate((float*)(arena+op[SE_gate]), arena+op[SE_expand], (signed char*)(wts+op[SE_fc1w]),
              (signed char*)(wts+op[SE_fc2w]), (float*)(wts+op[SE_blob]), (float*)(wts+op[SE_lut]),
              op[SE_Cexp],op[SE_Csq], op[SE_eH],op[SE_eW],op[SE_eWop], d32in, prof);
    } else if(op[0]==OP_SETAIL){
      setail_d32(arena+op[ST_out], arena+op[ST_conv], arena+op[ST_res], (float*)(arena+op[ST_gate]),
             (float*)(wts+op[ST_blob]), op[ST_C],op[ST_H],op[ST_W],op[ST_Wop],
             op[ST_zc],op[ST_zr],op[ST_zo],op[ST_relu],op[ST_hasres], op[ST_has_gate], op[ST_outd32], d32in);
    } else if(op[0]==OP_HEAD){
      // GAP (layout-specific) -> shared BN/quant/Gemm tail. Everything comes out of the shared scratch, dead by the
      // time the head runs: head arrays on the STACK took interp's frame from 9472 to 21248 B and blew the 16 KB
      // FastRPC thread stack, and statics are barred (they make .bss NOBITS). `gap` outlives the GAP (head_tail
      // reads it); the NHWC GAP's mcp/acc and head_tail's qu never coexist, so they share the block after it.
      int hC=op[HD_C], hO=op[HD_O], hHW=op[HD_HW];   // HD_HW is H*W for both layouts (lower.py packs it either way)
      unsigned char* hs=d32in; float* gap=SCR_TAKE(hs, hC*4);
      headp_t hp = head_prep((float*)(wts+op[HD_blob]), hC, hO, hHW);
      if(op[HD_d32])
        head_gap_d32(gap, arena+op[HD_conv], arena+op[HD_res], (float*)(arena+op[HD_gate]), &hp,
           hC,op[HD_H],op[HD_W],op[HD_Wop],op[HD_zcc],op[HD_zr],op[HD_hasres],op[HD_has_gate],op[HD_relu]);
      else
        head_gap_nhwc(gap, arena+op[HD_conv], arena+op[HD_res], (float*)(arena+op[HD_gate]), &hp,
           hC,hHW,op[HD_zcc],op[HD_zr],op[HD_hasres],op[HD_has_gate],op[HD_relu], hs);
      head_tail((float*)(arena+op[HD_out]), gap, &hp, (signed char*)(wts+op[HD_gw]), hC, hO, &pool, hs);
    } else if(op[0]==OP_DWCONV){
      // op[1]=out op[2]=in (NHWC arena); op[3]=filt op[4]=bias_sum (wts); op[5]=d32in op[6]=d32out (arena scratch); op[7..]=C,H,W,kh,kw,filt_zero,recip,shift
      dwconv_op(arena, wts+op[DW_filt], (int*)(wts+op[DW_bias]), minmax, op, &pool, prof);
    } else if(op[0]==OP_ADD){
      // QLinearAdd (residual): out = clip(round((sa*(a-za)+sb*(b-zb))/so)+zo, 0, qmax). Per-tensor scales, so the
      // pixel structure is irrelevant and this is literally a 2-step chain over the flat byte range -- spelled out
      // here rather than hidden behind an add_op() wrapper that only rebuilt these two lines.
      ewacc_t st[2]={{arena+op[ADD_a],op[ADD_za],op[ADD_ra]},{arena+op[ADD_b],op[ADD_zb],op[ADD_rb]}};
      ewise_op(arena+op[ADD_out], st, 2, op[ADD_S], op[ADD_zo], op[ADD_qmax], (long)op[ADD_n]);
    } else if(op[0]==OP_PACK){
      // entry seed pack: NHWC [H][W][Cin] -> d32 [H][Cin/32][Wop][32]
      pack_op(arena+op[PK_src], op[PK_W], arena+op[PK_out], op[PK_Wop]*32, op[PK_H], op[PK_Cigp], prof);
    } else if(op[0]==OP_INCONV){
      int inHo=op[IC_Ho];
      stemwork_t sb={arena+op[IC_out], arena+op[IC_src], (int*)(wts+op[IC_wvec]), (int*)(wts+op[IC_bias]),
                     (int*)(wts+op[IC_recip]), op[IC_Cin],op[IC_H],op[IC_W],op[IC_Cout],op[IC_k],op[IC_stride],
                     op[IC_pad],op[IC_zsh],op[IC_Ho],op[IC_Wo],0,inHo,0,2};
      sb.xzp=op[IC_xzp];   // PIL stem: fill the cp4 borders with the activation zero-point
      int split=inHo/2;                        // pool: main=[0,split), worker=[split,inHo)
      stemwork_t s0=sb, s1=sb; s0.oy0=0; s0.oy1=split; s1.oy0=split; s1.oy1=inHo;   // lock=0: pool worker holds HVX
      s0.cp4=arena+op[IC_cp4]; s1.cp4=arena+op[IC_cp4]+3*WP_PIL*4;   // per-worker cp4 arena slice
      pool_run2(&pool, stem_worker, &s0, &s1);
    }
    unsigned long long dt=HAP_perf_get_time_us()-t0;
    if(prof){ prof[op[0]] += dt; if(i<220) prof[16+i]=dt; }   // per-op time at prof[16+i]
  }
  if(pool.up) pool_stop(&pool);
}

// The on-DSP prefetch/vrmpy diagnostics live in pfcal.h -- ~240 lines of microbenchmark, compiled only for
// compile/pfcal.py. Their FINDINGS are recorded at the code they are about, not there.
#ifdef PF_CAL
#include "pfcal.h"
#endif

// ---- megakernel entry wrapper (tinygrad DSP custom-kernel convention) --------------------------------------------
// The whole backbone is ONE kernel: interp_kernel(out, seed, arena, wts, ops, scratch).
//
// This .so is MODEL-INDEPENDENT: build it once, run any model. The per-model layout used to arrive as 12 -D
// defines (NOPS, SEED_OFF, OUT_OFF, the scratch sub-offsets...), which quietly made the binary model-SPECIFIC even
// though the op list was data -- "model as data" was only half true. It now arrives as a HEADER RECORD at the
// front of the op stream, so the only thing that changes per model is the data.
#ifndef INTS_PER_OP
#error "define the op-record contract: INTS_PER_OP / OP_CONV / OP_SE_GATE / OP_SETAIL / OP_HEAD (codegen.opcode_defines)"
#endif
enum { HDR_NOPS, HDR_SEED_OFF, HDR_SEED_BYTES, HDR_OUT_OFF, HDR_OUT_BYTES,
       HDR_D32IN, HDR_D32OUT, HDR_MINMAX };   // ops[0..7]; must match megakernel.py's header()
static void interp_kernel(float* out, unsigned char* seed, unsigned char* arena, unsigned char* wts, int* ops,
                          unsigned char* scratch){
  const int* h = ops;
  int nops = h[HDR_NOPS], out_bytes = h[HDR_OUT_BYTES];
  unsigned char* d32in  = scratch + h[HDR_D32IN];
  unsigned char* d32out = scratch + h[HDR_D32OUT];
  int*           minmax = (int*)(scratch + h[HDR_MINMAX]);
  ops += INTS_PER_OP;                                   // step over the header record
  __builtin_memcpy(arena + h[HDR_SEED_OFF], seed, h[HDR_SEED_BYTES]);
#ifdef PROF_OPS
  // Dump per-op us into `out` (prof[16+i]). Reuse the OUTPUT buffer AS the prof array: no stack array (a 1KB stack
  // prof[] tips the near-limit resident/V65 call tree over -> rc=39) and no new .bss segment (`static` -> "too many
  // segments" at load). Needs out_bytes >= 8*(16+nops); the classifier out (1000B=125 slots) covers nops<=109.
  unsigned long long* prof = (unsigned long long*)out;
  int _pn = out_bytes/8; if(_pn>256) _pn=256; for(int i=0;i<_pn;i++) prof[i]=0;
#ifdef PF_CAL
  pf_calibrate(wts, d32in, prof); return;    // microbench only -- no model is run, `out` carries the results
#endif
  interp(arena, wts, ops, nops, d32in, d32out, minmax, prof);
#else
  interp(arena, wts, ops, nops, d32in, d32out, minmax, 0);
  __builtin_memcpy(out, arena + h[HDR_OUT_OFF], out_bytes);
#endif
}
// Single-kernel FastRPC entry, used by DSPProgram straight from the BINARY. Mirrors ops_dsp.py's
// DSPRenderer._render_entry: buffers arrive as offsets into shared ion arenas, so the entry maps the ARENAS, not each buffer.
//   pra[0]=[n_arenas, arena sizes...]  pra[1]=per-buffer 8-byte slot, (int arena, int offset)
//   pra[2]=timer (out)                 pra[3+j]=arena j's fd
// ★ THIS IS A CONTRACT WITH ops_dsp.py's DSPProgram.__call__, NOT with _render_entry: tinygrad renders no code for
// the megakernel (it arrives as Ops.BINARY), so the entry below is the only reader of that layout on this path.
// Changing the pra layout on the Python side without changing it HERE faults the PD -- every case, EPERM, no hint.
// HAP_munmap is not optional: the arenas are ION_FLAG_CACHED and unmapping is what makes the DSP's writes to `out`
// visible to the host -- dropping it (to save re-mapping ~15MB per call) returns cosine 0.14 on the first invoke.
int entry(unsigned long long handle, unsigned int sc, remote_arg* pra) {
  HAP_power_request_t req; for(unsigned i=0;i<sizeof(req);i++)((char*)&req)[i]=0;
  // EXACT replica of nnlib hexagon_nn_vote(TURBO): dcvs_enable=FALSE (pin corner, DCVS off) + PERFORMANCE_MODE policy
  // + set_latency + set_dcvs_params + min=max=target=CORNER, via NULL context. This precise combo pins the corner.
  req.type=HAP_power_set_DCVS_v2;
  req.dcvs_v2.dcvs_enable=0; req.dcvs_v2.dcvs_option=HAP_DCVS_V2_PERFORMANCE_MODE;
  req.dcvs_v2.set_latency=1; req.dcvs_v2.latency=100; req.dcvs_v2.set_dcvs_params=1;
  req.dcvs_v2.dcvs_params.max_corner=CORNER; req.dcvs_v2.dcvs_params.min_corner=CORNER; req.dcvs_v2.dcvs_params.target_corner=CORNER;
  HAP_power_set(0,&req);
  // Also vote MAX BUS BANDWIDTH: DCVS_v2 only sets the CORE corner, not the DSP<->DDR bus. Without this the memory-bound
  // ops are capped at ~400MB/s (SNPE hits ~4GB/s). set_mips_bw with a large bwBytePerSec @100% + min latency unthrottles it.
  HAP_power_request_t bw; for(unsigned i=0;i<sizeof(bw);i++)((char*)&bw)[i]=0;
  bw.type=HAP_power_set_mips_bw; bw.mips_bw.set_mips=1; bw.mips_bw.mipsPerThread=1000; bw.mips_bw.mipsTotal=4000;
  bw.mips_bw.set_bus_bw=1; bw.mips_bw.bwBytePerSec=12000000000ULL; bw.mips_bw.busbwUsagePercentage=100;
  bw.mips_bw.set_latency=1; bw.mips_bw.latency=1;
  HAP_power_set((void*)handle,&bw);
  if ((sc>>24) != 2) return 0;
  int na = ((int*)pra[0].buf.pv)[0];
  void *bases[16];
  for (int j = 0; j < na; j++) bases[j] = HAP_mmap(0, ((int*)pra[0].buf.pv)[1+j], 3, 0, pra[3+j].dma.fd, 0);
  void *b[6];
  for (int i = 0; i < 6; i++) b[i] = (char*)bases[((int*)pra[1].buf.pv)[i*2]] + ((int*)pra[1].buf.pv)[i*2+1];
  // CDSP core clock is HARD-PINNED at ~1.186 GHz (= 2 vrmpy/cyc = 303.5 GMAC/s, the compute ceiling) by the comma
  // system, below the entire client-accessible HAP layer. EXHAUSTIVELY tested (all left the peak UNCHANGED): DCVS
  // corners 2/4/6/7, dcvs_enable+PERFORMANCE_MODE, compute client class, HVX power-up, min=NOM/target=TURBO,
  // HAP_power_request_abs(1478/1305/998), maxed MIPS vote. Only the privileged SysMon/power-manager path could exceed
  // it. Device crash was the DEVFREQ governor sysfs write (update_devfreq null-PC) -- NOT any HAP call.
  unsigned long long start = HAP_perf_get_time_us();
  interp_kernel((float*)b[0], (unsigned char*)b[1], (unsigned char*)b[2], (unsigned char*)b[3], (int*)b[4], (unsigned char*)b[5]);
  *(unsigned long long *)(pra[2].buf.pv) = HAP_perf_get_time_us() - start;
  for (int j = 0; j < na; j++) HAP_munmap(bases[j], ((int*)pra[0].buf.pv)[1+j]);
  return 0;
}
