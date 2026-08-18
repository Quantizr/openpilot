"""The op list -> what the device actually receives: int32 op records, a resident weight blob, and the arena.

    arena_shapes(ops, ...)   every tensor's shape/dtype in CLOSED FORM, so a build never runs the model
    build_program(prog)      records + weight blob + arena/scratch sizes
    MK_RESIDENCY_DUMP=1      per-op layout decisions (the diagnostic, not a gate)

Everything between the op list and the device lives here, because none of it has any other consumer:

    the RECORD ABI     one field list per op kind -> `pack()` and megakernel.c's -D field offsets
    geometry           shapes and buffer sizes, derived EXACTLY ONCE (see the section header)
    the kernel REGISTRY  which HVX kernel runs an op, or a build error naming why none does
    quantization       float ONNX tensors -> the exact bytes each kernel reads
    residency          which tensors stay in d32 layout, and which depthwises get a padded buffer
    lowering           arena offsets, the weight blob, the op records

They were six files. Each was imported by exactly one other module -- this one -- which is the definition of
"part of it" rather than "a dependency of it".
"""
from __future__ import annotations

import os
import dataclasses
from dataclasses import dataclass, field

import numpy as np

from openpilot.selfdrive.modeld.dsp.compile.parse import emit, seed_quant



# ====================================================================================================================
# THE RECORD ABI -- declared once here; megakernel.c gets matching -D offsets
# was: codegen.py's first half
# ====================================================================================================================


INTS_PER_OP = 48   # the op record megakernel.c's interp() strides by; opcodes are op[0]. Reaches C as a
                   # single -D, so both sides move together -- grown from 32 when derived geometry moved
                   # host-side (C2): CONV already reached rec[31].
OP_CONV, OP_SE_GATE, OP_SETAIL, OP_HEAD = 1, 2, 3, 4
OP_DWCONV, OP_ADD, OP_INCONV = 5, 6, 8   # 7/9/10 were OP_GAP/OP_GEMM/OP_PEAK: no emitter ever existed
OP_PACK = 11   # entry pack: NHWC seed -> d32 seed buffer, so a d32 s0 stage's reduce+residual read it directly
# CONV: rec[6..24]=params, rec[25]=in_d32, rec[26]=out_d32, rec[27]=circ, rec[28]=bordered_Wp,
# rec[30]=bordered_base, rec[31]=in_left_skip. (dspbench megakernel conv_op contract.)


# ---- the op-record FIELD SCHEMA. Declared once here; the emitter indexes by name and megakernel.c gets matching
# -D defines, so `op[7]` (which means Cin for CONV but C for DWCONV, documented only in comments) stops being a
# thing that can silently drift. Same idea as LLVM's TableGen: one declaration, every consumer generated from it.
FIELDS: dict[str, list[str]] = {
  # DWCONV. rec[24..] is the geometry from dw_geom + the schedule from dw_sched (see C2).
  "DW": ["op", "out", "src", "filt", "bias", "d32in", "d32out", "C", "H", "W", "kh", "kw", "fz", "recip", "rsh",
         "aux", "s", "ind", "outd", "ind_bord", "in_lpad", "xzp", "_r22", "_r23",
         "Cp", "pad", "ofw", "padL", "Wp", "Hp", "oH", "oW", "oLp", "ils", "owt", "src_wop", "tile", "threads", "kern"],
  "ADD": ["op", "out", "a", "b", "ra", "rb", "S", "za", "zb", "zo", "qmax", "n"],
  "SE":  ["op", "gate", "expand", "blob", "fc1w", "fc2w", "lut", "Cexp", "HW", "Csq", "d32", "eH", "eW", "eWop"],
  "ST":  ["op", "out", "conv", "res", "gate", "blob", "C", "HW", "zc", "zr", "zo", "relu", "hasres", "has_gate",
          "d32", "H", "W", "Wop", "outd32"],
  "HD":  ["op", "out", "conv", "res", "gate", "blob", "gw", "C", "HW", "zcc", "zr", "hasres", "O", "has_gate",
          "d32", "H", "W", "Wop", "relu"],
  "PK":  ["op", "out", "src", "W", "H", "Cigp", "Wop"],
  # INCONV (the low-depth stem). Was the last op with NO schema at all -- raw rec[:19]/rec[:23] and a bare rec[23].
  "IC":  ["op", "out", "src", "wvec", "bias", "recip", "Cin", "H", "W", "Cout", "k", "stride", "pad", "zsh",
          "Ho", "Wo", "cp4", "xzp"],
  # CONV. rec[6..] is what conv_op receives as `p` (so p[i] == CV_x - 6).
  "CV":  ["op", "out", "src", "wt", "bias", "recip",
          "Cin", "H", "W", "Cout", "kh", "kw", "sh", "sw", "ph", "pw", "groups", "xzp", "zshift",
          "Ho", "Wo", "Wop", "Win", "Hp", "w_gcstride",
          "in_d32", "out_d32", "circ", "bord_Wp", "bord_base", "in_left_skip",
          "Cigp", "Cogp", "tot", "sz_d32in", "sz_d32out"],
}
CV_P0 = 6   # conv_op takes `op+CV_P0`, so its p[i] is FIELDS["CV"][i+CV_P0]

# ★ RESERVED IN EVERY RECORD, at the SAME index for every op kind: the bytes of the shared scratch region this op
# carves. build.scratch_layout takes ONE max over it, so an op that needs scratch cannot be forgotten in a per-kind
# elif chain -- which is exactly how this file's two worst bugs happened. Both were a fixed-size C array sized for
# one model's widths: `gqtab[2048]` (needs C*4 shorts, so C=768 ran 2KB past it) and `fc1u[128]` (needs Csq floats,
# so a 768-channel SE with Csq=192 ran 256 B past it, into the middle of interp's frame). A per-kind sizing table
# in build.py cannot be kept honest; a field every record carries can.
SCRATCH = INTS_PER_OP - 1


def field_defines() -> list[str]:
  d = [f"-D{pfx}_{nm}={i}" for pfx, names in FIELDS.items() for i, nm in enumerate(names) if not nm.startswith("_")]
  # conv_op is handed `op+CV_P0`, so it needs the SAME fields renumbered relative to that pointer
  d += [f"-DCV_P0={CV_P0}"]
  d += [f"-DP_{nm}={i - CV_P0}" for i, nm in enumerate(FIELDS["CV"]) if i >= CV_P0 and not nm.startswith("_")]
  return d


def scr(*sizes: int) -> int:
  """Total bytes of the shared scratch a kernel carves, given its sub-buffer sizes IN THE ORDER IT TAKES THEM.
  megakernel.c carves with SCR_TAKE, which 128-aligns each one (HVX aligned loads), so this rounds identically."""
  return sum(_ru(int(n), 128) for n in sizes)


def pack(pfx: str, *geoms, scratch: int = 0, **fields) -> list[int]:
  """One op record, built from NAMED fields -- the schema above is the only thing that knows an index.

  `geoms` are dataclasses (DwGeom / ConvGeom / DwSched) whose field names are deliberately the SAME as the
  schema's, so the derived-geometry tail of a record needs no per-field list at all; anything they carry that the
  schema does not name (nbytes_in, circ, ...) is simply not a record field and is dropped. Explicit `fields` win
  over `geoms`, which is what lets the bordered-project case override Win.

  This replaces a hybrid of `rec[:6] = [...]`, `rec[6:6+len(params)] = params` and `rec[F(pfx,name)] = ...` -- and
  with it the positional writes that were the last of P2: `params[16] = bp[0]` (index 16 of an unnamed list) is now
  `Win=bp[0]`, and build.py's `r[35]`/`r[37]` are `r[F("CV","sz_d32in")]`."""
  names = FIELDS[pfx]
  assert len(names) <= SCRATCH, f"{pfx}: schema reaches the reserved SCRATCH slot {SCRATCH}"
  d: dict = {}
  for g in geoms:
    d.update({k: v for k, v in dataclasses.asdict(g).items() if k in names})
  d.update(fields)
  if unknown := set(d) - set(names):
    raise KeyError(f"{pfx}: no such record field(s) {sorted(unknown)} -- schema is {names}")
  rec = [0] * INTS_PER_OP
  for k, v in d.items():
    rec[names.index(k)] = int(v)
  rec[SCRATCH] = int(scratch)
  return rec


def F(pfx: str, name: str) -> int:
  """Index of a named field, for the emitter. The C side reads the same index via the -D above."""
  return FIELDS[pfx].index(name)


def opcode_defines() -> list[str]:
  """The op-record contract, handed to megakernel.c as -D so the C and this encoder cannot drift."""
  return [f"-DINTS_PER_OP={INTS_PER_OP}", f"-DOP_CONV={OP_CONV}", f"-DOP_SE_GATE={OP_SE_GATE}",
          f"-DOP_SETAIL={OP_SETAIL}", f"-DOP_HEAD={OP_HEAD}", f"-DOP_DWCONV={OP_DWCONV}", f"-DOP_ADD={OP_ADD}",
          f"-DOP_INCONV={OP_INCONV}",
          f"-DOP_PACK={OP_PACK}", f"-DWP_PIL={WP_PIL}"] + field_defines()


def _ru(x, m):
  return (x + m - 1) // m * m


# ---- the PIL stem's cp4 gather buffer. ONE constant, TWO consumers: megakernel.c gets it as a -D and sizes every
# cp4 walk from it, and the arena allocation below sizes the buffer from it. It used to be `#define WP_PIL 288` in
# the C **and** a literal `2*3*288*4` in the emitter -- the C uses all scaled off the macro, the ALLOCATION did not,
# so raising the macro alone would have silently overrun the arena. Same one-formula-two-languages class as P1.
# Raised 288 -> 448 so a full-width 3-channel image stem (W=384) takes the stem path instead of falling to a dense
# conv, which pads Cin 3->32 and then needs an entry OP_PACK (measured 12.7ms on MobileNetV4 @384). Cost is arena:
# 2 workers x 3 taps x WP_PIL x 4B = 10.75KB, up from 6.9KB.
WP_PIL = 448            # padded cp4 width in RGBA words, mult 32
STEM_MAX_W = WP_PIL - 64   # the gather reaches input col 2*(Wo-1)+2 == W, plus the 64-word slack the fill assumes


def _pad32(ops):
  """Pad every conv's Cin/Cout to mult-32 (zero-filled) so ALL activation tensors are mult-32 -> full d32 residency
  + fast pack/unpack. Pad channels carry zero weights -> contribute 0; the real output (final Gemm, unpadded) is
  unchanged. Build-side only.

  ★ MEASURED 2026-07-26, because this pass BUYS speed by ADDING WORK and that trade had never been quantified.
  On MobileNetV2 it pads 9 of 51 ops (16->32, 24->32, 144->160) for **+15.5% MACs** (288.7M -> 333.4M), and it is
  worth it by a mile:

      with _pad32     5.49 ms          without    22.55 ms   (+308.7%, 4.1x)   cosine identical

  because the padding is what makes d32 RESIDENCY reachable. Unpadded, 6 convs and 2 depthwises fall back to NHWC
  and the whole pack/from_d32 roundtrip returns. distill is unaffected (already mult-32 everywhere).

  ⇒ This is also why megakernel.c has no scalar d32->NHWC unpack any more. The alternative to a non-mult-32
  shape is not "unpack it slowly", it is "pad it and use from_d32_asm", and that alternative is ALWAYS available
  since zero weights are always legal. Two cases this pass does NOT yet cover are asserted at emit rather than
  handled slowly -- see the conv and dwconv branches of build_program."""
  for o in ops:
    if o.t not in ("CONV", "DWCONV", "INCONV"):   # SE/SETAIL/HEAD follow the (padded) conv shapes
      continue
    w = o.w_q
    Cout, Cin = w.shape[0], w.shape[1]
    Coutp = _ru(Cout, 32)
    if o.t == "INCONV":                 # stem: pad Cout ONLY -- it reads its input as NHWC at the real depth, so
      if Coutp == Cout:                 # padding Cin would reintroduce the Cin->32 waste the stem exists to avoid
        continue                        # (and would break its own `Cin <= 4` predicate)
      nw = np.zeros((Coutp, Cin, w.shape[2], w.shape[3]), w.dtype)
      nw[:Cout] = w
      o.w_q = nw
    elif o.t == "DWCONV":               # depthwise: pad groups=Cout to mult-32 (new 1-channel groups, w=0)
      if Coutp == Cout:
        continue
      nw = np.zeros((Coutp, 1, w.shape[2], w.shape[3]), w.dtype)
      nw[:Cout] = w
      o.w_q, o.groups = nw, Coutp
    else:                               # CONV: pad Cout and (groups==1) Cin
      Cinp = _ru(Cin, 32) if o.groups == 1 else Cin
      if Coutp == Cout and Cinp == Cin:
        continue
      nw = np.zeros((Coutp, Cinp, w.shape[2], w.shape[3]), w.dtype)
      nw[:Cout, :Cin] = w
      o.w_q = nw
    ws = np.asarray(o.w_scale, np.float64).reshape(-1)
    if ws.size > 1:
      o.w_scale = np.concatenate([ws, np.full(Coutp - Cout, ws[-1])])
    if o.bias_f is not None:
      o.bias_f = np.concatenate([np.asarray(o.bias_f, np.float64).reshape(-1), np.zeros(Coutp - Cout)])


# ====================================================================================================================
# GEOMETRY -- shapes and buffer sizes, derived EXACTLY ONCE
# was: geometry.py
# ====================================================================================================================


"""Buffer geometry -- derived EXACTLY ONCE, here, and read by everyone else.

The rule this file exists to enforce:

    Python owns every formula that encodes a DECISION -- the `&~3` roundings, the `+8` slack, the padL choice,
    the ofw filter stride, the oLp/ils phase. C may only do arithmetic on values it was handed.

Before this, the depthwise formulas lived in THREE places (megakernel.c's dwconv_op, codegen's shared-scratch
sizing, and codegen's _dw_geom for the bordered path) and had to be kept in agreement by hand. Generalizing
stride-2 `padL` from a hardcoded 9 to `4*s+pad` on 2026-07-24 required editing two of them; missing one is a
silent wrong answer or an out-of-bounds write. Emitting every intermediate would not have helped -- `niw =
D*Wp*32` cannot disagree with itself -- so the line is drawn at policy, not arithmetic.

Consumers: the op-record emitter (codegen), the arena/scratch sizer (codegen, build.scratch_layout), the bordered
T1a planner, and -- via the record -- the kernel itself."""

# (_ru defined once above -- geometry, quant and lower each carried their own copy)
def out_hw(H, W, kh, kw, sh, sw, ph, pw):
  """THE convolution output size. Trivial, and repeated in four places before this: conv_geom, dw_geom,
  codegen's arena_shapes and its INCONV branch. Repeating a formula is how the padL bug in this file's header
  happened -- one copy got the fix and the others did not."""
  return (H + 2 * ph - kh) // sh + 1, (W + 2 * pw - kw) // sw + 1


@dataclass(frozen=True)
class DwGeom:
  """Depthwise buffer  `padL` is the only input that varies between the two call sites: the RESIDENT
  (T1a bordered) path forces 4 so the producing conv can write the valid region with a 128-byte-aligned store,
  while the non-resident path uses the natural pad. Everything else follows from it -- which is precisely why
  there must be one function and not two."""
  Cp: int; pad: int; ofw: int; padL: int
  Wp: int; Hp: int                 # input buffer [Hp][Cp/32][Wp][32]
  oH: int; oW: int                 # valid output dims
  oLp: int; ils: int; owt: int     # out_left_pad (junk cols the consumer skips), in_left_skip, padded out width
  nbytes_in: int; nbytes_out: int
  base_off: int                    # where the producer writes the valid region of the bordered buffer

  @property
  def D(self): return self.Cp // 32


def dw_geom(C, H, W, kh, kw, s, padL_override=0) -> DwGeom:
  """The one definition. `padL_override` = the resident path's in_left_pad (4); 0 means derive it.

  Stride-2 uses `padL = 4*s + pad`, which makes out_left_pad exactly 4 -- a multiple of 4, so the from_d32 read
  pointer (to + oLp*32) stays 128-byte aligned -- AND drives the in_left_skip term to zero identically for any k,
  which is what lets s2_5xN (no in_left_skip arg) work at all. k=3 gives 9, the value this was hardcoded to."""
  Cp, pad, ofw = _ru(C, 32), kh // 2, _ru(kw, 4)
  padL = padL_override or (4 * s + pad if s == 2 else pad)
  Wp = (_ru(W, 4) + ofw + 2 * padL + 8) & ~3
  Hp = H + 2 * pad + 2
  oH, oW = out_hw(H, W, kh, kw, s, s, pad, pad)
  oLp = (padL - pad) // s
  ils = ((padL - (oLp * s + pad)) & 1) * 8 if s == 2 else 0   # s1 has no in_left_skip arg (junk comes from col 0)
  owt = _ru(oW + oLp, 4)
  D = Cp // 32
  return DwGeom(Cp=Cp, pad=pad, ofw=ofw, padL=padL, Wp=Wp, Hp=Hp, oH=oH, oW=oW, oLp=oLp, ils=ils, owt=owt,
                nbytes_in=D * Hp * Wp * 32, nbytes_out=oH * D * owt * 32, base_off=pad * D * Wp * 32 + padL * 32)


@dataclass(frozen=True)
class ConvGeom:
  """Conv buffer  `Ho/Wo/Wop/Win/Hp` were already emitted in the op record; what was duplicated is
  everything derived AROUND them -- the channel-group padding in conv_op, the six scratch region sizes in
  build.scratch_layout, and the circular-buffer size in _v65_pick, each re-deriving from the others' outputs."""
  Cigp: int; Cogp: int; tot: int              # padded per-group channel counts and total out-chunks
  Ho: int; Wo: int; Wop: int; Win: int; Hp: int
  sz_d32in: int; sz_d32out: int             # scratch region sizes, in ELEMENTS
  circ: int                                   # V65 circular-datapath scratch, in bytes (0 if not V65)
  padL: int = 0                               # the input buffer's LEFT PAD in pixels (4 when pw>0, for
  ils: int = 0                                # 128-B aligned stores); ils = padL - pw, absorbed by repstream


CONV_NT = 2   # must match megakernel.c's CONV_NT (V65 conv row-split fan-out)


def conv_geom(Cin, Cout, H, W, kh, kw, sh, sw, ph, pw, groups, in_w_override=0) -> ConvGeom:
  """The one definition. `in_w_override` is the bordered-project case: the input is the dw's d32 output, so the
  circular buffer must be sized from that buffer's width (owt-oLp) rather than from this conv's own Win."""
  Cig, Cog = Cin // groups, Cout // groups
  Cigp, Cogp = _ru(Cig, 32), _ru(Cog, 32)
  Ho, Wo = out_hw(H, W, kh, kw, sh, sw, ph, pw)
  Wop = _ru(Wo, 4)
  Win = (Wop - 1) * sw + kw
  # The input-buffer CHUNK stride is Win*32 and the repstream does 128-B ALIGNED vmem loads, so Win must be a
  # multiple of 4 or the 2nd+ input chunk is read from a rounded-down (wrong) address. kw==1 gives Win==Wop
  # (already mult-4, which is why every 1x1 worked); kw>1 gives Wop+kw-1, which is not.
  # ★ LEFT PAD 4, NOT pw, WHENEVER THERE IS ONE. The producer of this buffer (conv_repad_d32, or pack) writes the
  # valid region at byte offset padL*32; with pw=1 that is 32 bytes into a 128-byte line, so EVERY vector store is
  # unaligned and conv_repad_d32 runs at 1.26 GB/s (0.855ms on distill, 9.4% of the model). Forcing the stores
  # aligned measures -46%, and two kernel-side rewrites failed to capture it because real rows straddle at both
  # ends -- the only way to remove the straddle is for the write to START 128-aligned. padL=4 does that, exactly
  # as dw_geom's padL_override=4 already does for the depthwise and for exactly this reason.
  # in_left_skip carries the difference: repstream reads aligned from column 0 and valign-shifts by it, so the
  # conv still sees pw columns of zero-point before the data.
  padL = 4 if pw > 0 else 0
  ils = padL - pw if pw > 0 else 0
  Win += ils
  circ_win = Win                              # the circ buffer is sized from the UNROUNDED Win
  Win = _ru(Win, 4)
  Hp = H + 2 * ph
  tot = groups * (Cogp // 32)
  in_width_pad = ((in_w_override or circ_win) + 3 + 8 * sw) & ~3
  # ★ ONE SLICE PER THREAD. megakernel.c's v65 path gives thread tt the slice at `circ_base + tt*slice`, where
  # slice = (buf_width*buf_height + 127) & ~127 -- exactly the term below. Sizing this for a single thread let any
  # THREADED conv write past the region into the next arena tensor. Latent until 2026-07-25: distill never started
  # the pool (no depthwise) so no conv threaded, and the shapes that did thread happened not to corrupt a live
  # tensor. Exposed immediately by conv2x2s2 (cosine 1.0 -> 0.9994985) once the pool was enabled for conv graphs.
  circ = ((in_width_pad * 2 * Cigp * max(kh, sh) + 127) & ~127) * CONV_NT + 256
  return ConvGeom(Cigp=Cigp, Cogp=Cogp, tot=tot, Ho=Ho, Wo=Wo, Wop=Wop, Win=Win, Hp=Hp, padL=padL, ils=ils,
                  sz_d32in=groups * Hp * (Cigp // 32) * Win * 32, sz_d32out=Ho * Wop * 32 * tot, circ=circ)


@dataclass(frozen=True)
class DwSched:
  """The Halide split: WHAT the depthwise computes is geometry, HOW it is executed is schedule. These were `if`s
  and `#define`s inside dwconv_op, so a new architecture silently inherited distill/MNv2's tuning. Deciding them
  host-side is also what lets C9 autotune them per model instead of hand-sweeping one global constant."""
  tile: int      # output rows per asm call (prefetch the next tile's d32in while this one computes)
  threads: int   # 1 or 2; 2 splits the tile loop across HVX units


def dw_sched(g: DwGeom, kh: int, s: int) -> DwSched:
  """Measured gates, moved verbatim from megakernel.c so this commit is behaviour-preserving.
  tile=4 swept best: -35% on mem-bound tall shapes (d32in > L2), neutral where d32in already fits L2.
  Threading is SIZE-GATED: 2 threads give ~1.9x DDR bandwidth but add barrier overhead, so it only pays on the
  big high-res stride-1 3x3 dws (measured: op1 C=32 112x112 0.362->0.322 WINS; the 56x56 and stride-2 ones LOSE,
  op7 +0.05, op4 +0.08). dw is memory-bound, so this is a bandwidth trade, not a compute one."""
  tile = int(os.getenv("DW_TH", "4"))
  thr = 2 if (kh == 3 and s == 1 and g.oH * g.oW >= int(os.getenv("DW_THREAD_MIN", "8192"))) else 1
  return DwSched(tile=tile, threads=thr)


# ====================================================================================================================
# KERNEL REGISTRY -- which kernel runs an op, or a loud build error
# was: registry.py
# ====================================================================================================================


"""Which kernel runs an op -- one declarative table, and a loud failure when nothing matches.

Kernel selection used to be scattered: `_v65_pick`'s env-gated levels in codegen, `have_asm`'s
`kh==3||kh==5||kh==7` conditions in megakernel.c, and a handful of build-time asserts bolted on as each
unsupported shape was discovered. Adding a kernel meant editing all three and remembering the asserts.

FAST-OR-FAIL (user decision 2026-07-24): there is no generic slow fallback. If no kernel matches, the BUILD fails
with a message naming the op and why -- we would rather refuse than silently ship a 13x-slower path. verify.py
found exactly that on its first run: stride-2 7x7 depthwise had no asm and quietly fell to the scalar
dw_mxn_cn at 5.0ms against 0.39ms for the stride-2 5x5. Genericity comes from the ewise VM (new activations are
chain data at full speed), not from slow C fallbacks.

Adding a kernel is now one row here plus the extern + dispatch arm in megakernel.c."""

# id -> (name, predicate on (kh, kw, s)). id 0 is reserved for "no kernel".
DW_KERNELS = [
  (1, "dwconv2dbbb_s1_3x3", lambda kh, kw, s: s == 1 and kh == 3 and kw == 3),
  (2, "dwconv2dbbb_s1_5xN", lambda kh, kw, s: s == 1 and kh == 5 and kw == 5),
  (3, "dwconv2dbbb_s1_7xN", lambda kh, kw, s: s == 1 and kh == 7 and kw == 7),
  (4, "dwconv2dbbb_s2_3x3", lambda kh, kw, s: s == 2 and kh == 3 and kw == 3),
  (5, "dwconv2dbbb_s2_5xN", lambda kh, kw, s: s == 2 and kh == 5 and kw == 5),
]


def pick_stem(Cin: int, Cout: int, kh: int, s: int, W: int) -> None:
  """Check the low-depth stem (OP_INCONV) shape, or raise. There is exactly ONE stem kernel -- the PIXELS-IN-LANES
  one (32 output pixels in lanes, i8 scalar-Rt weights, no vsplat: 269 GMAC/s, MobileNetV2's stem 0.995 -> ~0.45ms)
  -- so this is a validity check, not a choice. WP_PIL bounds the gather, hence W <= STEM_MAX_W (both above).

  ★ This found a real latent bug (2026-07-24). The emitter still had an `else` branch building a SECOND, differently
  laid out INCONV record for the old nnlib `inconv2dbbb332` kernel -- which was deleted from megakernel.c earlier in
  the campaign. `stem_worker` unconditionally calls stem_conv_pil, so any model whose stem missed the predicate
  would have had a PIL stem read that record's fields from the wrong slots: a SILENT WRONG ANSWER, exactly what
  fast-or-fail exists to stop. The dead branch and its five unused scratch allocations are gone; the shape check
  now fails the build."""
  if not (Cin <= 4 and Cout == 32 and kh == 3 and s == 2 and W <= STEM_MAX_W):
    raise AssertionError(
      f"no stem kernel for Cin={Cin} Cout={Cout} {kh}x{kh} stride {s} width {W}. stem_conv_pil handles Cin<=4, "
      f"Cout==32, 3x3 stride 2, W<={STEM_MAX_W}. Raise WP_PIL (one constant, both consumers follow) or let the "
      "shape fall to the ordinary conv path in parse.py -- but do NOT emit an OP_INCONV record it cannot read.")


def pick_dw(kh: int, kw: int, s: int) -> int:
  """The depthwise kernel id for this shape, or a build error. Also subsumes the asymmetric/even-k asserts that
  used to be written out separately: neither can match a row, so both fail here with the same message."""
  for kid, _name, pred in DW_KERNELS:
    if pred(kh, kw, s):
      return kid
  raise AssertionError(
    f"no depthwise kernel for {kh}x{kw} stride {s}. Available: " +
    ", ".join(f"{n.split('_', 1)[1]}" for _i, n, _p in DW_KERNELS) +
    ". Add the nnlib .S + a row in DW_KERNELS + a dispatch arm in dwconv_op, or change the model. "
    "(There is deliberately no scalar fallback -- it would run ~13x slower with no warning.)")


# ====================================================================================================================
# QUANTIZATION -- float ONNX tensors -> the bytes each HVX kernel reads
# was: quant.py
# ====================================================================================================================


"""Weights and scales: float ONNX tensors -> the exact bytes each HVX kernel reads.

    requant(...)        THE fixed-point requant every conv path shares (see its docstring)
    conv_params(...)    the V65 d32 weight layout + biasbuf/recip + the op record's conv params
    dw_params(...)      the depthwise-asm filter layout + per-channel bias/recip
    inconv_params(...)  the pixels-in-lanes stem's [go][ky][kx][32][4] weight layout

Split out of codegen.py, which had grown into four unrelated jobs (this, the ONNX->op-list emitter, the
op-list->record lowering, and the record ABI). Nothing here knows about arenas, layouts or op records -- it is
pure "what bytes does this kernel want", which is why it is separable at all."""

FILT_ZERO = 128   # the dw-asm's filter zero point: weights are stored as int8 + 128


# (_ru defined once above -- geometry, quant and lower each carried their own copy)
def requant(x_scale, w_scale, out_scale, out_zp, bias_f, wsum, x_zp, n):
  """THE fixed-point requant, shared by every conv path (conv / depthwise / stem). Was written out three times.

  Folds the float multiplier M = x_scale*w_scale/out_scale into an int32 `recip` at shift `rsh`, and folds BOTH
  the output zero point and the ACTIVATION zero-point correction into the integer bias, so the kernel does one
  integer multiply-shift and no per-pixel zp pass. `wsum` is the signed weight sum per output channel (the
  caller supplies it -- the V65 conv gets it from the weight rearrangement's gemsumb, the others sum directly).

  Per-TENSOR and per-CHANNEL w_scale are the same code: a scalar is broadcast, and `rsh` comes from M.max() so a
  per-channel recip cannot overflow int32. That equivalence is why there is one function here and not three.

  This is also the ONE place the device's requant rounding is defined -- worth knowing, because the residual
  ~1e-4 cosine against the float ONNX is this fixed point, not a bug (measured 2026-07-24, see the memory)."""
  ws = np.asarray(w_scale, np.float64).reshape(-1)
  if ws.size == 1:
    ws = np.full(n, ws[0])
  M = x_scale * ws / out_scale
  rsh = max(0, int(np.ceil(np.log2(max(float(M.max()), 1e-12)))) + 1)
  recip = np.round(M * (2.0 ** (31 - rsh))).astype(np.int64)
  bias_q = np.round((bias_f if bias_f is not None else np.zeros(n)) / (x_scale * ws)).astype(np.int64)
  if out_zp:
    bias_q = bias_q + np.round(out_zp / M).astype(np.int64)
  return bias_q - x_zp * np.asarray(wsum, np.int64), recip, rsh

def inconv_params(w_q, w_scale, bias_f, x_scale, x_zp, out_scale, out_zp, kh, kw):
  """Stem conv Cin<=4, 3x3, s2 -> (weights, bias_q[Cout], recip, rsh). PER-TENSOR w_scale. Weight layout
  [go][ky][kx][32][4] = raw int8, read as a scalar Rt pair by stem_conv_pil's vrmpy. ic>=Cin -> 0; bias folds
  bias_q - x_zp*Sigma_w so the xzp-filled borders cancel (activation-zp corr).

  The `pil` flag is gone with the nnlib inconv2dbbb332 kernel it selected (deleted earlier in the campaign, and
  registry.pick_stem already fails the build for anything the PIL stem cannot take) -- one stem, one layout."""
  Cout, Cin = w_q.shape[0], w_q.shape[1]
  wsum = w_q.reshape(Cout, -1).astype(np.int64).sum(1)                  # signed weight sum per out-channel
  bias_q, recip, rsh = requant(x_scale, w_scale, out_scale, out_zp, bias_f, wsum, x_zp, Cout)
  bias_q, recip = bias_q.astype(np.int32), int(recip[0])                # stem requant is per-TENSOR
  wp = np.zeros((Cout, 4, kh, kw), np.int8)                            # ic >= Cin reads 0 (weight[3] always 0)
  wp[:, :Cin] = w_q
  return wp.reshape(Cout // 32, 32, 4, kh, kw).transpose(0, 3, 4, 1, 2).copy(), bias_q, recip, rsh

def dw_pack(w_q, kh, kw):
  """w_q int [C,1,kh,kw] SIGNED -> uint8 filt in the nnlib dwconv2dbbb layout (bit-exact, from dspbench dw_ref).
  filt[32*d*kh*ofw + ofw*fy*32 + z*4 + 128*(fx//4) + (fx%4)], ofw=(kw+3)&~3; real taps fx<kw = w+128, pad taps = 0.
  C padded to mult-32: pad channels get w=0 (-> stored 128 for real taps -> zum cancels -> outputs 0).

  That index is AFFINE in (d, fy, fx//4, z, fx%4) with strides (kh*ofw*32, ofw*32, 128, 4, 1) -- i.e. the layout is
  just [C/32][kh][ofw/4][32][4] viewed row-major, so this is a transpose, not a loop. Written out as five nested
  Python loops it was also the slowest thing in a build after the oracle."""
  Cp, ofw = _ru(w_q.shape[0], 32), (kw + 3) & ~3
  w = np.zeros((Cp, kh, kw), np.int64)
  w[: w_q.shape[0]] = w_q.reshape(w_q.shape[0], kh, kw)
  filt = np.zeros((Cp, kh, ofw), np.uint8)             # pad taps fx>=kw stay 0 (NOT FILT_ZERO)
  filt[:, :, :kw] = (w + FILT_ZERO) & 0xFF
  return filt.reshape(Cp // 32, 32, kh, ofw // 4, 4).transpose(0, 2, 3, 1, 4).ravel()

def dw_params(w_q, w_scale, bias_f, x_scale, x_zp, out_scale, out_zp, kh, kw):
  """Depthwise conv -> (filt, bias_sum[Cp], recip, rsh) for the dw-asm. PER-TENSOR w_scale. C padded to mult-32."""
  C = w_q.shape[0]
  Cp = _ru(C, 32)
  wsum = w_q.reshape(C, -1).astype(np.int64).sum(1)        # signed weight sum per channel
  bias_q, recip, rsh = requant(x_scale, w_scale, out_scale, out_zp, bias_f, wsum, x_zp, C)
  recip = int(recip[0])                                    # dw requant is per-TENSOR
  bias = np.zeros(Cp, np.int32)
  bias[:C] = bias_q.astype(np.int32)                       # pad channels -> 0 bias -> 0 output
  return dw_pack(w_q, kh, kw), bias, recip, rsh, Cp


# ---- the conv's host side: the d32 weight layout the asm kernel reads and its fixed-point requant
# params, both of which SHIP in the weight blob. The numpy replica of the MAC is reference.host_conv_ref.

def rearrange_d32_v65(w_q):
  """w_q[Cout,Cin,kh,kw] int8 SIGNED -> (flat repacked signed-as-u8 blob, gemsumb[Coutp]). SAME tile layout as
  rearrange_d32 but the stored coeff is the SIGNED weight (two's-complement byte, V65 `vrmpy .b` reads it as int8)
  and padding channels are 0. gemsumb = Σ signed weight per out-channel (pad -> 0), which folds into the V65 biasbuf
  as the activation-zero-point correction (no per-pixel suma). Matches supernode_procweights.c signed_mode_sel=1."""
  Cout, Cin, kh, kw = w_q.shape
  Coutp, Cinp = _ru(Cout, 32), _ru(Cin, 32)
  wp = np.zeros((kh, kw, Cinp, Coutp), np.int16)                       # signed, pad = 0 (not filt_offset)
  wp[:, :, :Cin, :Cout] = w_q.transpose(2, 3, 1, 0).astype(np.int16)
  gemsumb = wp.astype(np.int32).sum(axis=(0, 1, 2))                    # Σ per out-channel -> [Coutp] signed
  # The tile index is AFFINE in (X, y, D, z, V, s, i) -- out_chunk, ky, in_chunk, kx, quad, out_lane, in_quad-lane
  # -- with strides (32*kh*kw*Cinp, 32*kw*Cinp, 1024*kw, 1024, 128, 4, 1), i.e. that 7-tuple viewed row-major. So
  # the seven nested loops are one transpose of wp re-viewed as [y][z][D][V][i][X][s]. MEASURED on distill's
  # largest conv (512->512 3x3): 2142 ms of Python loops -> 25 ms, an 85x speedup, output bit-identical. This runs
  # once per conv (56 of them in distill), so it was by a wide margin the slowest pure-Python left in a build.
  out = (wp.reshape(kh, kw, Cinp // 32, 8, 4, Coutp // 32, 32).transpose(5, 0, 2, 1, 3, 6, 4).ravel() & 0xFF)
  return out.astype(np.uint8), gemsumb

def conv_params(x_q_shape, x_scale, x_zp, w_q, w_scale, bias_f, kh, kw, sh, sw, ph, pw, groups, out_scale, out_zp):
  Cin, H, W = x_q_shape
  Cout = w_q.shape[0]
  g = conv_geom(Cin, Cout, H, W, kh, kw, sh, sw, ph, pw, groups)   # ONE definition; see the GEOMETRY section header
  Cig, Cog = Cin // groups, Cout // groups
  Cogp, totchunks = g.Cogp, g.tot
  Ho, Wo, Wop, Win, Hp = g.Ho, g.Wo, g.Wop, g.Win, g.Hp
  # Signed weights + gemsumb: the weight rearrangement yields the per-channel weight sum, and requant() folds it
  # into the bias as the activation-zp correction -- which is why there is no per-pixel suma pass.
  reps, sumbs = zip(*(rearrange_d32_v65(w_q[gi * Cog : (gi + 1) * Cog].reshape(Cog, Cig, kh, kw).astype(np.int32))
                      for gi in range(groups)))
  wsum = np.concatenate([sb[:Cog] for sb in sumbs])
  bias_q, recip_pc, zsh = requant(x_scale, w_scale, out_scale, out_zp, bias_f, wsum, x_zp, Cout)
  biasbuf, recip = np.zeros(groups * Cogp, np.int32), np.zeros(groups * Cogp, np.int32)
  for gi in range(groups):   # scatter Cout -> the Cogp-PADDED per-group layout the kernel indexes
    biasbuf[gi * Cogp : gi * Cogp + Cog] = bias_q[gi * Cog : (gi + 1) * Cog]
    recip[gi * Cogp : gi * Cogp + Cog] = recip_pc[gi * Cog : (gi + 1) * Cog]
  wrep = np.concatenate(reps)
  w_gcstride = wrep.size // totchunks
  # Ho/Wo/Wop/Win/Hp and the Cigp/Cogp/tot/sz_* tail are NOT listed here: they are ConvGeom fields, and pack()
  # takes them straight off `g` by name. Only what this function actually derives is returned.
  # ★ the record's `pw` is the BUFFER's left pad (g.padL), not the conv's semantic padding; in_left_skip carries
  # the difference so repstream lands the window correctly. Both writers of the buffer (pack, conv_repad_d32) use
  # it purely as a layout offset, which is what makes the substitution safe.
  params = dict(Cin=Cin, H=H, W=W, Cout=Cout, kh=kh, kw=kw, sh=sh, sw=sw, ph=ph, pw=(g.padL or pw), groups=groups,
                xzp=x_zp, zshift=zsh, w_gcstride=w_gcstride, in_left_skip=g.ils)
  return wrep, biasbuf, recip, params, (Cout, Ho, Wo), g


# ====================================================================================================================
# RESIDENCY -- which tensors stay in d32, and the T1a bordered depthwises
# was: residency.py
# ====================================================================================================================


"""Which tensors stay in d32 layout, and which depthwises get a persistent padded buffer.

    R = plan(ops, shp, seed_name, alloc, off)     annotates each op in place; returns what is not per-op
    MK_RESIDENCY_DUMP=1                           per-op decision table (the diagnostic, not a gate)

d32 is [H][C/32][W][32]. Every op boundary that changes layout costs a pack or a from_d32 transpose -- measured
at 22ms/43% of MobileNetV2 before any of this existed, and 3.75ms of distill. So the planner's job is to keep a
whole stage resident and pay the transpose only at stage boundaries.

★★★ THE FIXPOINT IS A **GREATEST** FIXPOINT, AND THAT IS NOT AN IMPLEMENTATION DETAIL.
Every candidate tensor starts `d32=True` and the iteration only ever RETRACTS:

    ok = len(c) >= 1 and _prod_std(nm) and all(_reads_d32(x, nm) for x in c)

Write it the other way round -- start False and add -- and you get a LEAST fixpoint, which produces correct
output with silently worse performance: fewer tensors resident, more transposes, and no test failure anywhere.
That is also why this pass is NOT written with tinygrad's graph_rewrite (which computes a least fixpoint) even
though C6's fusions are: the direction is wrong, and the edges here are ROLE-NAMED dict fields (`src`, `conv`,
`res`, `a`, `b`, `expand`) rather than positional UOp src -- `_reads_d32` genuinely needs to know whether a name
arrived as a SETAIL's `conv` or its `res`.

★ `len(c) >= 1` IS A VACUOUS-TRUTH GUARD, NOT A FORMALITY. `all()` over an empty consumer list is True, so a
tensor with NO consumers would be declared d32 and its producer would write a layout nobody reads. This bites
under MK_TRUNC, where the truncated terminal op has zero consumers.

★ THE MUTUAL DEPENDENCY IS REAL. `_reads_d32` consults the in-progress `d32t` for SETAIL/HEAD (setail_d32 cannot
mix layouts, so `conv` and `res` must BOTH be d32 -- each one's answer depends on the other's) and for ADD. A
predicate reading the fixpoint state mid-iteration is exactly what makes this a fixpoint and not a single pass."""

_ROLES = ("src", "expand", "conv", "res", "gate", "a", "b")   # every field by which one op names another


# The per-op layout DECISIONS this pass writes onto each op dict. They live on the op because that is what they
# describe; they were `id(op)`-keyed side tables only because nothing was writing them down anywhere else, which
# then forced every reader through `R.io(o)` / `id(o) in R.se`. `_pad32` already mutates ops in place.
#   in_d32/out_d32  CONV/DWCONV/INCONV: read (write) the input (output) in d32
#   d32             ADD/SE_GATE/SETAIL/HEAD: this op runs on d32 buffers
#   out_d32         SETAIL only: it ALSO writes d32 (stage interior); its in and out are independent
ANNOTATIONS = ("in_d32", "out_d32", "d32")


@dataclass
class Residency:
  """What is left of the layout plan once the per-op booleans live on the ops: the BORDERED buffers, which carry
  real geometry (an arena offset, a DwGeom) rather than a flag, plus the seed decision and the settled tensor
  set for the dump. Those legitimately stay a side table -- they are not properties of a single op."""
  seed_d32: bool = False                            # the seed is packed to d32 at entry (OP_PACK)
  bord_exp: dict = field(default_factory=dict)      # id(expand) -> (offset, Wp, base_off)
  bord_dw: dict = field(default_factory=dict)       # id(dw)     -> (offset, DwGeom)
  bord_proj: dict = field(default_factory=dict)     # id(project)-> (owt, oLp)
  d32t: dict = field(default_factory=dict)          # the settled tensor set (kept for the dump)


def plan(ops, shp, seed_name, alloc, off) -> Residency:
  """`alloc`/`off` are passed in because the T1a planner has to ALLOCATE the dw's bordered buffer and re-point
  the expand's output at it -- it is a layout decision with an allocation consequence, not a pure analysis."""
  R = Residency()
  for o in ops:                       # a re-plan (build_program runs twice for the lifetime allocator) must not
    for a in ANNOTATIONS:             # inherit the previous pass's decisions
      if hasattr(o, a):
        setattr(o, a, 0)
  cons: dict[str, list] = {}
  for o in ops:
    for key in _ROLES:
      nm = getattr(o, key, None)   # dynamic ROLE name -- the one place a field is not a literal
      if isinstance(nm, str):
        cons.setdefault(nm, []).append(o)
  prod = {o.out: o for o in ops if isinstance(o.out, str)}

  def _set(op, ind=None, outd=None):
    if ind is not None:
      op.in_d32 = ind
    if outd is not None:
      op.out_d32 = outd

  def _conv_std(o):
    """Does this op write STANDARD d32 that a d32 consumer can read directly?"""
    if o.t == "CONV":   # per-group Cog%32==0; a depthwise-as-grouped conv (Cog=1) gives unreadable 1-ch chunks
      return o.w_q.shape[0] % 32 == 0 and (o.w_q.shape[0] // o.groups) % 32 == 0
    if o.t == "DWCONV":   # s1 only: s2's output has oLp junk-left columns a plain consumer cannot skip
      return o.w_q.shape[0] % 32 == 0 and o.sh == 1
    if o.t == "INCONV":
      # ★ AT Cout==32, NHWC AND d32 ARE THE SAME BYTES. d32 is [H][C/32][W][32]; at C==32 the chunk axis has
      # extent 1 and it collapses to [H][W][32], which is exactly the NHWC [H][W][C] the PIL stem writes -- so
      # long as the d32 row width Wop==Wo, i.e. Wo%4==0. registry.pick_stem already pins Cout==32, so this is
      # really only the Wo%4 test. The stem needs no change: its consumer just stops packing.
      # This case was MISSING, so _conv_std fell through to False and MobileNetV2's first depthwise ran the
      # scalar NHWC->d32 pack() on the stem's output for nothing. It was the last live pack() caller in either
      # gate model. NOTE the op gets out_d32=1 annotated but the IC record carries no out_d32 field and
      # stem_conv_pil always writes NHWC -- correct precisely BECAUSE the two layouts coincide here, which is
      # why the Cout==32 guard is load-bearing rather than decorative.
      Cout, _Ho, Wo = shp[o.out][0]
      return Cout == 32 and Wo % 4 == 0
    return False

  def _reads_d32(c, nm):
    """Can consumer op `c` read tensor `nm` as d32? Dispatches on the ROLE `nm` plays for `c`, which is why this
    is not a structural pattern match."""
    if c.t == "SE_GATE" and c.expand == nm:
      return True                       # gap_accum_d32 is a plain per-channel GAP -- layout-invariant
    if c.t in ("SETAIL", "HEAD") and (c.conv == nm or c.res == nm):
      # setail_d32/head_d32 cannot MIX layouts, so conv and res must both be d32 -- reading one as d32 requires
      # the other to be d32 too. Hence the lookup into the in-progress d32t (see the header).
      if not c.res:
        # A RES-LESS HEAD is a plain GAP+Gemm classifier (MNv2): it must NOT read d32, so the last conv unpacks
        # to NHWC and head_hvx reads it contiguously. ★ RE-MEASURED 2026-07-26 (per-op, within-run, not an A/B),
        # because "d32 is faster" holds nearly everywhere else in this pipeline and it is worth knowing why not
        # here. Forcing MNv2's head d32: the head goes 370 -> 880us (2.4x SLOWER) while its producing conv saves
        # only 65us by skipping the from_d32; net +445us, +8.5% on the model. The reason is the shape -- at
        # C=1280, HW=7x7, head_d32 walks 40 depth chunks x Wop=8 columns for 7 valid ones, while head_hvx reads
        # one contiguous [49][1280]. This is the one place the d32 default loses, and it is not close.
        return c.t == "SETAIL"
      return bool(R.d32t.get(c.res if c.conv == nm else c.conv, False))
    if c.t in ("CONV", "DWCONV") and c.src == nm:   # kind FIRST: only a Conv has .src
      # 1x1 reads d32 directly; k>1 REPADs it (border-fill + contiguous copy, no transpose), and a dw repads too.
      # Both are yes, so there is no branch here -- there used to be one, left over from the MK_REPAD3 ablation.
      return True
    if c.t == "ADD" and (c.a == nm or c.b == nm):
      return R.d32t.get(c.out, False)   # an add reads d32 iff the add itself is d32 (== its own out)
    return False

  seed_d32_ok = shp[seed_name][0][0] % 32 == 0    # the entry OP_PACK needs mult-32 channels

  def _prod_std(nm):
    if nm == seed_name:
      return seed_d32_ok
    if (p := prod.get(nm)) is None:
      return False
    if _conv_std(p):
      return True
    if p.t == "ADD":
      return R.d32t.get(p.a, False) and R.d32t.get(p.b, False)
    if p.t == "SETAIL":   # setail_d32 is layout-invariant: it writes std d32 iff its conv (+res) are d32
      return R.d32t.get(p.conv, False) and (not p.res or R.d32t.get(p.res, False))
    return False

  # ---- the greatest fixpoint. START OPTIMISTIC, REFINE DOWN. See the header before changing this. -------------
  cand = [o.out for o in ops if isinstance(o.out, str) and (_conv_std(o) or o.t in ("ADD", "SETAIL"))]
  if seed_d32_ok:
    cand.append(seed_name)
  R.d32t = {nm: True for nm in cand}
  changed = True
  while changed:
    changed = False
    for nm in cand:
      c = cons.get(nm, [])
      ok = len(c) >= 1 and _prod_std(nm) and all(_reads_d32(x, nm) for x in c)   # len>=1: vacuous-truth guard
      if R.d32t.get(nm) != ok:
        R.d32t[nm] = ok
        changed = True

  # ---- settled -> per-op flags ------------------------------------------------------------------------------
  for o in ops:
    nm = o.out
    if isinstance(nm, str) and R.d32t.get(nm):
      if o.t == "ADD":
        o.d32 = 1
      elif o.t != "SETAIL":   # SETAIL below: its in and out d32 are INDEPENDENT decisions
        _set(o, outd=1)
    if o.t in ("CONV", "DWCONV") and R.d32t.get(o.src):
      _set(o, ind=1)
    if o.t == "SE_GATE" and R.d32t.get(o.expand):
      o.d32 = 1
    if o.t == "SETAIL" and R.d32t.get(o.conv) and (not o.res or R.d32t.get(o.res)):
      o.d32 = 1
      if R.d32t.get(o.out):
        o.out_d32 = 1
    if o.t == "HEAD" and o.res and R.d32t.get(o.conv) and R.d32t.get(o.res):
      o.d32 = 1   # residual head only -- see the res-less note in _reads_d32
  R.seed_d32 = bool(R.d32t.get(seed_name, False))

  # ---- T1a BORDERED depthwise residency (SNPE-style persistent-padded d32) ----------------------------------
  # For expand(1x1, mult-32) -> dw(k>=3) -> project(1x1): the expand writes its valid region ALIGNED into the
  # dw's padded [Hp][D][Wp][32] buffer (in_left_pad=4), the dw reads it in place (ind_bordered, no repad) and
  # writes d32 out at out_left_pad=oLp, and the project's repstream valign-absorbs oLp. Kills the dw's repad AND
  # the project's from_d32/pack.
  for dw in ops:
    if dw.t != "DWCONV" or dw.kh < 3 or dw.w_q.shape[0] % 32:
      continue
    exp = prod.get(dw.src)   # the expand must be the dw's SOLE producer and feed ONLY the dw
    if (exp is None or exp.t != "CONV" or exp.kh != 1 or exp.kw != 1 or exp.groups != 1
        or exp.w_q.shape[0] % 32 or cons.get(exp.out) != [dw]):
      continue
    dc = cons.get(dw.out, [])
    if len(dc) != 1:
      continue
    proj = dc[0]
    if proj.t != "CONV" or proj.kh != 1 or proj.kw != 1 or proj.groups != 1 or proj.src != dw.out:
      continue
    C, (H, W) = dw.w_q.shape[0], shp[dw.src][0][1:]
    # padL_override=4 so the producing conv can write the valid region with a 128-byte-aligned store. One
    # definition of the geometry -- see the GEOMETRY section header for why that matters.
    g = dw_geom(C, H, W, dw.kh, dw.kh, dw.sh, padL_override=4)
    boff = alloc(exp.out + "_bord", ((g.nbytes_in,), np.uint8))
    off[exp.out] = boff   # re-point the expand's output at the bordered buffer
    R.bord_exp[id(exp)] = (boff, g.Wp, g.base_off)
    R.bord_dw[id(dw)] = (boff, g)
    R.bord_proj[id(proj)] = (g.owt, g.oLp)
    _set(exp, outd=1)
    _set(dw, ind=1, outd=1)
    _set(proj, ind=1)

  if os.getenv("MK_RESIDENCY_DUMP"):
    _dump(ops, R)
  return R


def _dump(ops, R: Residency) -> None:
  """Per-op decision table. Residency changes otherwise surface only as a timing delta or a cosine break, with no
  way to see WHICH op flipped -- which is what made this pass frightening to touch."""
  print(f"{'#':>3} {'op':8} {'in_d32':>6} {'out_d32':>7} {'bordered':>8}  out")
  for i, o in enumerate(ops):
    # Field presence VARIES BY KIND (B2): Conv has in_d32/out_d32; Setail has d32+out_d32; SeGate/Add/Head have
    # only d32. This whole function was left on the pre-B2 dict API and crashed on the first SeGate.
    ind, outd = getattr(o, "in_d32", ""), getattr(o, "out_d32", "")
    bord = ("exp" if id(o) in R.bord_exp else "dw" if id(o) in R.bord_dw else
            "proj" if id(o) in R.bord_proj else "")
    # `t` exists only on Conv (it selects the kernel); the other kinds are named by their class. `d32` exists on
    # SeGate/Setail/Add/Head but NOT Conv, which uses in_d32/out_d32 -- hence getattr on both.
    extra = ("d32" if getattr(o, "d32", 0) else "")
    print(f"{i:>3} {getattr(o, 't', type(o).__name__):8} {ind:>6} {outd:>7} {bord:>8}  {o.out} {extra}")


# ====================================================================================================================
# LOWERING -- arena, weight blob, op records
# was: lower.py
# ====================================================================================================================


LUTN = 4096
LUT_LO, LUT_HI = -16.0, 16.0
LUT_SCALE = LUTN / (LUT_HI - LUT_LO)  # 128.0


def sigmoid_lut():
  u = LUT_LO + (np.arange(LUTN) + 0.5) / LUT_SCALE
  return (1.0 / (1.0 + np.exp(-u))).astype(np.float32)


_LUT = sigmoid_lut()

def _circ_bytes(o, src_shape, in_w_override=0):
  """V65 circular-datapath scratch for a conv. EVERY conv runs V65 -- k==1 g==1, 3x3-s1, grouped and 3x3-s2 are all
  proven bit-exact on it, and it is what took distill_vision 19.3->15.7ms. The V60 gvconv path it replaced (a
  per-pixel `suma` correction plus its ds/II integral images) was deleted 2026-07-24 along with three of the six
  conv scratch regions; there is no longer a slow twin to fall back to, which is the fast-or-fail policy from C5."""
  Cin, H, W = src_shape
  return conv_geom(Cin, o.w_q.shape[0], H, W, o.kh, o.kw, o.sh, o.sw, o.ph, o.pw, o.groups, in_w_override).circ


def arena_shapes(ops, seed_name, seed_shape):
  """name -> (shape, dtype) for every arena tensor, in closed form -- so a build never runs the model.
  Verified against reference.run_arena (shape, dtype AND nbytes) for all 92 ops of distill_vision."""
  shp = {seed_name: (tuple(seed_shape), np.uint8)}
  for o in ops:
    if o.t in ("CONV", "DWCONV", "INCONV"):
      shp[o.out] = ((o.w_q.shape[0], *out_hw(*shp[o.src][0][1:], o.kh, o.kw,
                                                            o.sh, o.sw, o.ph, o.pw)), np.uint8)
    elif o.t == "SE_GATE":
      shp[o.gate] = ((shp[o.expand][0][0],), np.float32)   # gate[Cexp]
    elif o.t == "ADD":
      shp[o.out] = (shp[o.a][0], np.uint8)                 # elementwise residual add
    elif o.t == "SETAIL":
      shp[o.out] = (shp[o.conv][0], np.uint8)              # elementwise on the conv
    elif o.t == "HEAD":
      shp[o.out] = ((o.gw.shape[0],), np.float32)          # Gemm -> emb
  return shp

def build_program(prog, _preassigned=None):
  """Encode the op list for the C interpreter: arena offsets, weight blob, op records.
  Returns dict with ops(int32 [nops,INTS_PER_OP]), wts(bytes), arena_size, seed(off,shape), out(off,size),
  op_outoff (per-op output arena offset), op_list."""
  ops = emit(prog)[0]
  _pad32(ops)   # pad conv Cin/Cout to mult-32 -> all tensors mult-32 -> full residency (build side; reference stays real)
  if os.getenv("MK_TRUNC"):   # DEBUG: keep only the first N ops; the model output becomes op[N-1]'s tensor (per-op diff)
    ops = ops[:int(os.getenv("MK_TRUNC"))]
  _s, _z, seed_name = seed_quant(prog)
  shp = arena_shapes(ops, seed_name, prog["seed_shape"])
  def AL(n):  # 128-byte align (gvconv reads weights/activations as 128B HVX vectors)
    return (n + 127) // 128 * 128
  # ---- arena offset per tensor. Pass 1 bump-allocates and records every request; build() then computes tensor
  # lifetimes from the op list and re-runs with `_preassigned` offsets that REUSE dead tensors' space (C8).
  off, sizes, order = {}, {}, []
  cur = 0

  def alloc_out(o, d32_elems=None):
    """This op's output slice: the d32 byte count when it writes d32 (which is >= the NHWC size, so the NHWC
    shape would under-reserve), else the plain NHWC shape. Every op kind needs this and each used to spell out
    its own two-branch version."""
    return alloc(o.out, ((d32_elems,), np.uint8) if d32_elems is not None else shp[o.out])

  def alloc(name, sd):
    nonlocal cur
    if name in off:
      return off[name]
    shape, dt = sd
    nbytes = AL(int(np.prod(shape)) * np.dtype(dt).itemsize)
    sizes[name] = nbytes
    order.append(name)
    if _preassigned is not None and name in _preassigned:
      off[name] = _preassigned[name]
      cur = max(cur, off[name] + nbytes)
    else:
      off[name] = cur
      cur += nbytes
    return off[name]

  alloc(seed_name, shp[seed_name])
  # ---- V65 selection (booleans). The shared circular-datapath scratch is sized AFTER the residency + bordered planners
  #   below: a bordered-dw consumer (project) reads owt-wide rows -> needs a bigger circ slice than its own Wo would give.
  # ---- LAYOUT: which tensors stay d32, and which depthwises get a persistent padded buffer. The RESIDENCY section owns
  # the whole decision (a greatest fixpoint plus the T1a bordered planner) and returns it as ONE object; it used
  # to be ~95 lines here producing ten loose locals. `alloc`/`off` go in because T1a allocates. ----
  R = plan(ops, shp, seed_name, alloc, off)
  seed_is_d32 = R.seed_d32
  # ---- circular-datapath scratch, sized over every V65 conv (a bordered project reads its wider in_win=owt rows) ----
  max_circ = 0
  for o in ops:
    if o.t != "CONV":
      continue
    src_shape = (o.w_q.shape[1] * o.groups,) + shp[o.src][0][1:]
    # A bordered project reads the dw's WIDER owt rows, so its circ buffer is sized from that width -- which is
    # exactly what conv_geom's `in_w_override` is for. This used to re-derive the formula inline and dropped the
    # `* CONV_NT` per-thread multiplier in the process, so a threaded bordered project could write thread 1's
    # slice past the region (latent: max_circ is a max over ALL convs, so another conv's correct sizing covered it).
    bp = R.bord_proj.get(id(o))
    max_circ = max(max_circ, _circ_bytes(o, src_shape, (bp[0] - bp[1]) if bp else 0))
  circ_off = alloc("__v65_circ__", ((max_circ,), np.uint8)) if max_circ else 0
  # ---- shared DEPTHWISE scratch: the dw-asm's d32in/d32out/aux, reused across sequential dw ops (sized to max) ----
  dw_in_max = dw_out_max = dw_dmax = 0
  for o in ops:
    if o.t != "DWCONV":
      continue
    C = o.w_q.shape[0]
    g = dw_geom(C, shp[o.src][0][1], shp[o.src][0][2], o.kh, o.kw, o.sh)
    dw_dmax = max(dw_dmax, g.D)
    dw_in_max, dw_out_max = max(dw_in_max, g.nbytes_in), max(dw_out_max, g.nbytes_out)
  dw_d32in_off = alloc("__dw_d32in__", ((dw_in_max,), np.uint8)) if dw_in_max else 0
  dw_d32out_off = alloc("__dw_d32out__", ((dw_out_max,), np.uint8)) if dw_out_max else 0
  # dw aux = D recip vectors (the 5xN/7xN per-channel-scale reads) + the 64-int min/max + the 7xN ~2KB sbuf
  dw_aux_off = alloc("__dw_aux__", ((max(64 * 128, dw_dmax * 128 + 4096),), np.uint8)) if dw_in_max else 0
  # ---- weight blob ----
  blob = bytearray()

  def put(arr, dt):
    if len(blob) % 128:
      blob.extend(b"\0" * (128 - len(blob) % 128))  # gvconv reads weights as 128B HVX vectors
    o = len(blob)
    blob.extend(np.ascontiguousarray(arr, dt).tobytes())
    return o

  lut_off = put(_LUT, np.float32)
  # s0 SEED d32: allocate the d32-seed buffer + emit an entry OP_PACK (NHWC seed -> d32). s0's reduce reads it in_d32
  # and the first SETAIL's residual reads it, so the whole s0 stage goes d32. seed_C mult-32 (gated above).
  seed_d32_off = None
  if seed_is_d32:
    sC, sH, sW = shp[seed_name][0]
    seed_d32_off = alloc("__seed_d32__", ((_ru(sC, 32) // 32 * sH * _ru(sW, 4) * 32,), np.uint8))
  def _rd(name, is_d32):   # redirect a d32 consumer of the seed to the packed d32-seed buffer
    return seed_d32_off if (seed_is_d32 and name == seed_name and is_d32) else off[name]

  recs = []
  op_outoff = []

  def _pack_before(o, src_name):
    """Insert an OP_PACK converting `src_name` from NHWC to plain d32, and return the packed buffer's offset.

    ★ LAYOUT CONVERSION IS AN OP, NOT A CASE INSIDE EVERY KERNEL. This is what lets every compute kernel read d32
    and ONLY d32. Without it each kernel needs an NHWC twin, which is how this file used to carry a scalar pack()
    at 2.33 ns/B plus NHWC copies of setail and the SE GAP -- three implementations of "the other layout".

    It exists because some producers CANNOT write standard d32: a stride-2 depthwise (its d32 output has oLp junk
    left columns no plain consumer can skip), a grouped conv with Cog%32, and the conv feeding a res-less HEAD
    (measured: NHWC is 2.4x faster there). Their consumers still want d32, so the mismatch has to be paid
    somewhere -- and paying it once, here, with the vectorized to_d32_asm, beats paying it in every kernel.

    ★ MEASURED REGRESSION THIS FIXES (2026-07-27). Deleting the NHWC paths earlier today was justified by "no gate
    model executes this", which is the wrong test: the bar is the mnv2/distill op set in any REASONABLE order, not
    the two graphs we happen to own. `dw_s2 -> conv3x3` and `dw_s2 -> dw_s1` are both reasonable and both stopped
    building. They build again, now without a scalar path anywhere."""
    nonlocal recs, op_outoff
    C, H, W = shp[src_name][0]
    Cigp, Wop = _ru(C, 32), _ru(W, 4)
    d32_off = alloc(src_name + "_d32", ((Cigp // 32 * H * Wop * 32,), np.uint8))
    if src_name not in packed:   # one pack per tensor, even when several consumers need it
      recs.append(pack("PK", op=OP_PACK, out=d32_off, src=off[src_name], W=W, H=H, Cigp=Cigp, Wop=Wop))
      op_outoff.append(d32_off)
      packed.add(src_name)
    return d32_off

  packed: set = set()
  for o in ops:
    if o.t == "CONV":
      in_d32, out_d32 = o.in_d32, o.out_d32
      in_off = _rd(o.src, in_d32)
      wrep, bb, rc, params, _, cg = conv_params(
        (o.w_q.shape[1] * o.groups,) + shp[o.src][0][1:], o.x_scale, o.x_zp, o.w_q, o.w_scale, o.bias_f,
        o.kh, o.kw, o.sh, o.sw, o.ph, o.pw, o.groups, o.out_scale, o.out_zp)
      # FAST-OR-FAIL: an NHWC-emitting conv is unpacked by from_d32_asm, which needs whole 32-chunks per group.
      # _pad32 pads Cout to mult-32 but NOT to mult-(32*groups), so a grouped conv can still land here. The fix is
      # to widen _pad32, not to add back a scalar unpack -- see its docstring for the 4.1x that decides this.
      if not out_d32 and (o.w_q.shape[0] // o.groups) % 32:
        raise NotImplementedError(
          f"conv {o.out}: groups={o.groups} gives Cog={o.w_q.shape[0] // o.groups}, not a multiple of 32, and this "
          f"conv must emit NHWC. from_d32_asm cannot unpack it. Fix: pad Cout to a multiple of 32*groups in _pad32.")
      # NHWC input that to_d32_asm cannot take verbatim (anything but 1x1 s1 unpadded) gets an inserted OP_PACK --
      # see _pack_before. conv_op then repads the plain d32 exactly as it does for any other d32 producer.
      if not in_d32 and not (o.groups == 1 and o.kh == o.kw == 1 and o.ph == o.pw == 0
                             and o.sh == o.sw == 1 and o.w_q.shape[1] % 32 == 0):   # shape[1] is Cin/groups
        in_off, in_d32 = _pack_before(o, o.src), 1
      be, bp = R.bord_exp.get(id(o)), R.bord_proj.get(id(o))
      out_off = alloc_out(o, cg.tot * cg.Ho * cg.Wop * 32 if out_d32 else None)   # d32 [Ho][totchunks][Wop][32]
      # The derived-geometry tail comes off ConvGeom by NAME (see pack). A bordered project reads the dw's wider owt
      # rows, so re-derive with that input width -- only `circ` actually differs, but re-deriving is the rule.
      g2 = cg if not bp else conv_geom(
        o.w_q.shape[1] * o.groups, o.w_q.shape[0], *shp[o.src][0][1:], o.kh, o.kw, o.sh,
        o.sw, o.ph, o.pw, o.groups, in_w_override=bp[0] - bp[1])
      f = dict(op=OP_CONV, out=out_off, src=in_off, wt=put(wrep, np.uint8), bias=put(bb, np.int32),
               recip=put(rc, np.int32), in_d32=int(in_d32), out_d32=int(out_d32), circ=circ_off, **params)
      if be:   # T1a: write the d32 output into the dw's bordered [Hp][D][Wp][32] buffer
        f |= dict(bord_Wp=be[1], bord_base=be[2])       # chunk width, byte offset of the first valid pixel
      if bp:   # the V65 repstream reads the dw's owt-wide rows and valign-skips its out_left_pad
        f |= dict(Win=bp[0], in_left_skip=bp[1])
      rec = pack("CV", g2, scratch=g2.sz_d32in, **f)
      op_outoff.append(out_off)
    elif o.t == "DWCONV":   # dspbench dwconv_op contract (op[7]=real C; megakernel pads to mult-32 internally)
      # ★ _rd, NOT off[] -- the seed's d32 consumers must be pointed at the OP_PACK'd buffer. This branch used
      # plain off[o.src], so a DEPTHWISE reading the seed directly got the RAW NHWC pointer while its record said
      # ind=1: it read NHWC bytes as d32. The CONV branch has always used _rd; the dw branch never did.
      # Neither gate model reaches it (MobileNetV2's first dw consumes the stem, distill has no depthwise), and
      # a full-model cosine could not have found it anyway -- see the note in verify.py's chain probes.
      in_off = _rd(o.src, o.in_d32)
      C = o.w_q.shape[0]
      s = o.sh
      kern = pick_dw(o.kh, o.kw, s)   # fast-or-fail: no match => build error, never a slow path
      in_d32, out_d32 = o.in_d32, o.out_d32
      filt, bias, recip, rsh, Cp = dw_params(o.w_q, o.w_scale, o.bias_f, o.x_scale, o.x_zp,
                                             o.out_scale, o.out_zp, o.kh, o.kw)
      H, W = shp[o.src][0][1:]
      bd = R.bord_dw.get(id(o))
      d32in_off, ind_bord, in_lpad = dw_d32in_off, 0, 0
      if bd:   # T1a: read the producer-written bordered buffer directly (no repad); write d32-out at owt (padL=4 geom)
        boff, g = bd
        d32in_off, ind_bord, in_lpad = boff, 1, 4
        out_off = alloc_out(o, g.nbytes_out)
      else:     # s1 d32-out: [H][Cp/32][owt][32] straight to `out` (owt=ru(W,4), oLp=0)
        out_off = alloc_out(o, (Cp // 32) * H * ((W + 3) & ~3) * 32 if out_d32 else None)
      # GEOMETRY (what) and SCHEDULE (how) are separate dataclasses, both named to match the schema -- see
      # see the GEOMETRY section. The kernel used to re-derive all of this from C/H/W/kh/s and agree with the sizing by hand.
      # FAST-OR-FAIL: same reason as the conv above, plus from_d32_asm over-writes when there is only ONE depth
      # chunk, so an NHWC-emitting depthwise needs C >= 64. Padding a 32-channel dw to 64 would double its work,
      # which is why this is an assert rather than something _pad32 does unconditionally.
      if not out_d32 and (C % 32 or C // 32 < 2):
        raise NotImplementedError(
          f"dwconv {o.out}: C={C} must emit NHWC, but from_d32_asm needs C mult-32 with >=2 depth chunks (C>=64). "
          f"Fix: keep this dw's output d32, or pad C to 64 in _pad32 (doubles its work -- measure first).")
      # A depthwise always needs a BORDER-padded d32 input, and to_d32_asm makes no borders -- so an NHWC producer
      # gets an inserted OP_PACK to plain d32, and the dw's own `ind` path border-pads it. See _pack_before.
      if not in_d32 and not ind_bord:
        in_off, in_d32 = _pack_before(o, o.src), 1
      gg = dw_geom(C, H, W, o.kh, o.kw, s, padL_override=in_lpad)
      sc = dw_sched(gg, o.kh, s)
      rec = pack("DW", gg, sc, op=OP_DWCONV, out=out_off, src=in_off, filt=put(filt, np.uint8),
                 bias=put(bias, np.int32), d32in=d32in_off, d32out=dw_d32out_off, C=C, H=H, W=W,
                 kh=o.kh, kw=o.kw, fz=FILT_ZERO, recip=recip, rsh=rsh, aux=dw_aux_off, s=s,
                 ind=int(in_d32), outd=int(out_d32), src_wop=_ru(W, 4), kern=kern,
                 ind_bord=ind_bord,     # the producer wrote the interior -> border-zap only, no repad
                 in_lpad=in_lpad,       # in_left_pad override (4 = resident, 128-byte aligned)
                 # ★ THE BORDER FILL VALUE. A conv's padding is zero in the DEQUANTIZED domain, which is
                 # u8 == the activation zero point, NOT u8 zero. dwconv_op filled its borders with literal
                 # zero and the deleted pack() was called with xzp=0 hardcoded, so a depthwise reading a
                 # tensor with a nonzero zero point computed the wrong thing at every edge pixel. It never
                 # fired on either gate model because every dw input there follows a Relu->Q (zp 0); a
                 # composition probe with zp=128 on the input found it at cosine 0.983 vs 0.9998.
                 xzp=o.x_zp)
      op_outoff.append(out_off)
    elif o.t == "INCONV":   # depth<=4 stem. FAST path = PIL (op[16]=2, dspbench's 5.8ms stem); else nnlib (op[17]=1).
      out_off = alloc(o.out, shp[o.out])
      in_off = off[o.src]
      Cout, Cin = o.w_q.shape[0], o.w_q.shape[1]
      H, W = shp[o.src][0][1:]
      pad = o.kh // 2
      Ho, Wo = out_hw(H, W, o.kh, o.kw, o.sh, o.sw, pad, pad)
      pick_stem(Cin, Cout, o.kh, o.sh, W)   # fast-or-fail; there is only the PIL stem
      # PIXELS-IN-LANES stem: 32 out-pixels/lanes, i8 scalar-Rt weights (no vsplat). xzp != 0 is handled by the
      # border fill plus the bias's -xzp*Sigma_w term (see inconv_params).
      wv, bias, recip, rsh = inconv_params(o.w_q, o.w_scale, o.bias_f, o.x_scale, o.x_zp,
                                           o.out_scale, o.out_zp, o.kh, o.kw)
      recipv = np.full(Cout, recip, np.int32)   # stem_conv_pil requants per-channel: broadcast the per-tensor recip
      rec = pack("IC", op=OP_INCONV, out=out_off, src=in_off, wvec=put(wv, np.int8), bias=put(bias, np.int32),
                 recip=put(recipv, np.int32), Cin=Cin, H=H, W=W, Cout=Cout, k=o.kh, stride=o.sh, pad=pad,
                 zsh=rsh, Ho=Ho, Wo=Wo, xzp=o.x_zp,
                 cp4=alloc(o.out + "_pilcp4", ((2 * 3 * WP_PIL * 4,), np.uint8)))   # 2 workers x 3 taps x WP_PIL*4
      op_outoff.append(out_off)
    elif o.t == "SE_GATE":
      Cexp = shp[o.expand][0][0]
      HW = int(np.prod(shp[o.expand][0][1:]))
      Csq = o.fc1_w.shape[0]
      # ★ FOLLOW THE EXPAND'S PADDING. _pad32 widens the expand conv's Cout to a multiple of 32 (e.g. RepViT's
      # 80 -> 96) but knows nothing about the SE that gates it, so fc2's per-output-channel rows/bias/scale stay
      # at the ORIGINAL width and every downstream reshape to (Cexp, Csq) fails. Pad them to match, with ZERO
      # weights: a pad channel's conv output is exactly its zero point (zero weights AND zero bias, see _pad32),
      # so `gate * (conv - zc)` is 0 there whatever the gate says. distill/MNv2 never hit this -- their SE widths
      # are already mult-32.
      # fc1 maps Cexp -> Csq, so the expand's padding adds INPUT COLUMNS to it. Zero columns: whatever a pad
      # channel's squeeze value is, it then contributes nothing.
      _f1 = o.fc1_w.reshape(Csq, -1)
      if _f1.shape[1] < Cexp:
        o.fc1_w = np.concatenate([_f1, np.zeros((Csq, Cexp - _f1.shape[1]), _f1.dtype)], axis=1)
      _n = o.fc2_w.shape[0] if o.fc2_w.ndim > 1 else o.fc2_w.size // Csq
      if _n < Cexp:
        o.fc2_w = np.concatenate([o.fc2_w.reshape(_n, Csq), np.zeros((Cexp - _n, Csq), o.fc2_w.dtype)])
        if o.fc2_b is not None:
          o.fc2_b = np.concatenate([np.asarray(o.fc2_b, np.float64).reshape(-1), np.zeros(Cexp - _n)])
        _ws = np.asarray(o.fc2_ws, np.float64).reshape(-1)
        if _ws.size > 1:
          o.fc2_ws = np.concatenate([_ws, np.full(Cexp - _n, _ws[-1])])
      # ★ ANY MULT-32 WIDTH. se_gate's fc2 and sigmoid run in whole 128-lane blocks, so the WEIGHTS are padded to
      # a mult-128 row stride here and the kernel derives the same stride. That is all the "masked tail" needs to
      # be: the trailing block computes garbage in the pad lanes, and nothing reads them -- the gate readback
      # loops to Cexp, and mAi/mBi are zero there so the lanes stay quiet. It was a BUILD ERROR for any width that
      # is not a multiple of 128, i.e. 3 of every 4 widths _pad32 can produce (only 8 of 32 in 32..1024).
      # ★★ THE PADDING IS NOT OPTIONAL EVEN AT MULT-128 WIDTHS -- it is what makes the row stride 128-ALIGNED.
      # dot_u8i8 and the fc2 loop both read weights with ALIGNED vector loads (HVX_Vector*), so a row stride that
      # is only mult-32 makes row 1 onward misaligned, which on Hexagon FAULTS rather than reading slowly.
      # There is no longer a width CEILING here: se_gate's acc/xqu/fc1u moved off the stack into the declared
      # scratch below, which is sized from Cexp/Csq. (It was `Cexpp <= 1536`, and it never covered fc1u at all.)
      Cexpp = _ru(Cexp, 128)
      if Cexp % 32:
        raise NotImplementedError(f"SE_GATE expand width Cexp={Cexp} (op {o.out}): must be a multiple of 32 "
                                  f"(_pad32 guarantees this); the d32 GAP chunks by 32.")
      gate_off = alloc(o.gate, shp[o.gate])
      # An NHWC expand gets an inserted OP_PACK rather than an NHWC twin of the GAP (see _pack_before).
      exp_off = _pack_before(o, o.expand) if not o.d32 else off[o.expand]
      g0 = o.sc_exp / HW
      g1 = o.sc_exp * o.zc_exp
      # ★ BROADCAST M1 TO Csq. fc1_ws is length Csq for a per-CHANNEL quantized fc1 but length 1 for a
      # per-TENSOR one, and M1 inherits that length -- while the device reads Csq floats from this slot
      # (b1q = M1 + Csq). A length-1 M1 therefore shifts EVERY field after it in the blob by Csq-1 floats, so
      # mAi/mBi/Ssig come from garbage, the fixed-point sigmoid index saturates at 4095, and lut[4095] =
      # sigmoid(16) ~= 1.0 -- the SE block runs SILENTLY UNGATED. distill quantizes fc1 per-channel so it never
      # hit this; onnxruntime's quantize_static with per_channel=False produces exactly this shape.
      # Found 2026-07-27: device output matched a gate==1.0 model at 96.78% and the correct-gate model at 34.14%.
      M1 = np.broadcast_to((o.s_rm * o.fc1_ws / o.fc1_outs).astype(np.float32), (Csq,))
      b1q = np.round((o.fc1_b if o.fc1_b is not None else np.zeros(Csq)) / (o.s_rm * o.fc1_ws)).astype(np.float32)

      # float32 FIRST, then widen -- these no longer ride in the blob, but mAi/mBi are derived from them and the
      # f32 rounding is part of the value the device sees. Computing in f64 throughout shifts mAi by an LSB and
      # moves the sigmoid LUT index by one cell (measured: distill cosine 0.9999332 -> 0.9999290).
      A2 = (o.fc2_ins * o.fc2_ws).astype(np.float32)
      B2 = (o.fc2_b if o.fc2_b is not None else np.zeros(Cexp)).astype(np.float32)
      wsum1 = o.fc1_w.reshape(Csq, -1).astype(np.float64).sum(axis=1).astype(np.float32)  # Σw per fc1 out (vrmpy zp corr)
      # The device reads this blob at FIXED Csq strides, so a SHORT field silently misaligns everything after it
      # -- which is exactly how the per-tensor M1 above went unnoticed. Check rather than trust.
      for _nm, _a in (("M1", M1), ("b1q", b1q), ("wsum1", wsum1)):
        if len(_a) != Csq:
          raise AssertionError(f"SE_GATE {o.gate}: blob field {_nm} has length {len(_a)}, expected Csq={Csq}")
      # Sigmoid index in INT32 fixed point (vectorized on-device instead of the per-channel scalar-float chain).
      # idx = (int)((A2*acc2 + B2 + 16)*128) = (mA*acc2 + mB) with mA=128*A2, mB=128*(B2+16); bake mAi=round(mA*2^S),
      # mBi=round(mB*2^S) so device does (mAi*acc2 + mBi) >> S. S is chosen per op from an emit-time bound on |acc2|
      # (|acc2[c]| <= 255*||fc2_w[c,:]||_1) so mAi*acc2 never overflows int32. NEAR-exact: fixed-point idx may be +-1
      # at a LUT-cell boundary (negligible on a smooth sigmoid). mAi/mBi are stored in acc2's DEINTERLEAVED lane
      # order so the device multiply is elementwise; the scalar gather then applies the same {0,2,1,3} readback.
      mA = 128.0 * A2.astype(np.float64)
      mB = 128.0 * (B2.astype(np.float64) + 16.0)
      accbound = 255.0 * np.abs(o.fc2_w.reshape(Cexp, Csq).astype(np.float64)).sum(axis=1).max()
      denom = np.abs(mA).max() * accbound + np.abs(mB).max() + 1.0
      S2 = int(np.clip(np.floor(np.log2((2.0**31 - 1) / denom)) - 1, 0, 30))
      mAi = np.clip(np.round(mA * (2.0**S2)), -2**31, 2**31 - 1).astype(np.int32)
      mBi = np.clip(np.round(mB * (2.0**S2)), -2**31, 2**31 - 1).astype(np.int32)

      # acc2's lane order is the {0,2,1,3} double-widen permutation within each 128-block -- closed form, so it
      # scatters in one indexing op rather than a per-channel Python loop. A bijection, hence a plain scatter.
      c = np.arange(Cexp)
      lane = c & 127
      rr = lane & 3
      gg = np.where(rr == 1, 2, np.where(rr == 2, 1, rr))
      di = (c & ~127) + (gg << 5) + (lane >> 2)
      mAi_d, mBi_d = np.zeros(Cexpp, np.int32), np.zeros(Cexpp, np.int32)   # pad lanes stay 0 -> idx 0, unread
      mAi_d[di], mBi_d[di] = mAi, mBi
      sig = np.concatenate([mAi_d.view(np.float32), mBi_d.view(np.float32), np.array([S2], np.int32).view(np.float32)])
      # A2/B2 are NOT in the blob: they only ever fed a scalar fc2 fallback that no model reached, and the
      # device consumes their integerized form (mAi/mBi in `sig`) instead. See megakernel.c's se_gate.
      blob_off = put(np.concatenate([[g0, g1, 1.0 / o.s_rm, float(o.z_rm)], M1, b1q, wsum1, sig]), np.float32)
      def _padrows(w):   # [Csq][Cexp] -> [Csq][Cexpp]: aligned vector loads need a mult-128 row stride
        z = np.zeros((Csq, Cexpp), np.int8); z[:, :Cexp] = w; return z
      fc1w_off = put(_padrows(o.fc1_w.reshape(Csq, -1)), np.int8)
      fc2w_off = put(_padrows(o.fc2_w.reshape(Cexp, Csq).T), np.int8)  # [Csq][Cexp] for output-parallel vmpyiacc
      d32 = o.d32   # d32 expand: gap_accum_d32 reads [Ho][Cexp/32][Wop][32] (valid W)
      eH, eW = shp[o.expand][0][1:]
      rec = pack("SE", op=OP_SE_GATE, gate=gate_off, expand=exp_off, blob=blob_off, fc1w=fc1w_off, fc2w=fc2w_off,
                 lut=lut_off, Cexp=Cexp, HW=HW, Csq=Csq,
                 scratch=scr(Cexp // 32 * 128 * 4, Cexpp * 4, Cexp, Csq * 4),   # se_gate: atab, acc, xqu, fc1u

                 **(dict(d32=1, eH=eH, eW=eW, eWop=_ru(eW, 4)) if d32 else {}))
      op_outoff.append(gate_off)
    elif o.t == "ADD":   # OP_ADD: flat-vectorized residual add (dspbench add_core; setail_hvx is scalar for C<128)
      C, H, W = shp[o.a][0]
      d32 = o.d32   # d32: operate on the padded d32 byte range so a resident chain does not break here
      n = ((C + 31) // 32) * H * ((W + 3) & ~3) * 32 if d32 else C * H * W
      out_off = alloc_out(o, n if d32 else None)
      ma, mb = o.sa / o.so, o.sb / o.so
      smx, S = max(ma, mb, 1.0), 15
      while smx * (1 << S) >= 32760.0:
        S -= 1
      ra, rb = int(round(ma * (1 << S))), int(round(mb * (1 << S)))
      rec = pack("ADD", op=OP_ADD, out=out_off, a=off[o.a], b=off[o.b], ra=ra, rb=rb, S=S,
                 za=o.za, zb=o.zb, zo=o.zo, qmax=255, n=n)
      op_outoff.append(out_off)
    elif o.t == "SETAIL":
      C, H, W = shp[o.conv][0]
      HW = H * W
      # setail_d32's gqtab used to be a fixed 4 KB stack array bounding C to 512 (640 corrupted the stack
      # silently, 768 faulted the PD). It now lives in the shared scratch, declared by the `scratch=` field
      # below -- so there is no channel ceiling here any more. What DOES still hold is d32-chunking:
      assert C % 32 == 0, f"SETAIL at {o.out}: C={C} is not a multiple of 32 (nchunk=C/32 truncates)"
      d32 = o.d32          # reads d32 conv/res (use setail_d32)
      # NHWC inputs get inserted OP_PACKs rather than an NHWC twin of the whole kernel (see _pack_before). What
      # lands here is a stride-2 depthwise feeding a SETAIL: its d32 output carries oLp junk left columns, so it
      # can only hand over NHWC. setail_d32 already writes EITHER layout, so only the input ever needed fixing.
      if not d32:
        conv_off, res_off, d32 = _pack_before(o, o.conv), (_pack_before(o, o.res) if o.res else 0), 1
      else:
        conv_off, res_off = _rd(o.conv, d32), (_rd(o.res, d32) if o.res else 0)
      outd32 = o.out_d32   # also writes d32 (interior); else writes NHWC (stage exit)
      out_off = alloc_out(o, _ru(C, 32) // 32 * H * _ru(W, 4) * 32 if outd32 else None)
      gate_off = off[o.gate] if o.gate else 0
      hasres = 1 if o.res else 0
      blob_off = put([o.sc / o.so, o.sr / o.so], np.float32)
      rec = pack("ST", op=OP_SETAIL, out=out_off, conv=conv_off, res=res_off, gate=gate_off, blob=blob_off,
                 C=C, HW=HW, zc=o.zc, zr=o.zr, zo=o.zo, relu=int(o.relu), hasres=hasres,
                 has_gate=int(o.has_gate), scratch=scr(C * 8))   # setail_d32: gqtab[C*4 shorts]
      if d32:   # rec[14]=use-setail_d32, rec[15..17]=Ho,Wo,Wop, rec[18]=out_d32
        rec[F("ST", "d32")], rec[F("ST", "H")], rec[F("ST", "W")] = 1, H, W
        rec[F("ST", "Wop")], rec[F("ST", "outd32")] = _ru(W, 4), int(outd32)
      op_outoff.append(out_off)
    elif o.t == "HEAD":
      Cc, Hh, Ww = shp[o.conv][0]
      C = Cc
      HW = Hh * Ww
      O = o.gw.shape[0]
      hd32 = o.d32
      out_off = alloc(o.out, shp[o.out])
      conv_off = _rd(o.conv, hd32)
      gate_off = off[o.gate] if o.gate else 0
      res_off = _rd(o.res, hd32) if o.res else 0
      hasres = 1 if o.res else 0
      P = (o.bn_scale / HW).astype(np.float32)       # gap_sum * P + Q; parse folded the BN, this folds the /HW
      Q = o.bn_shift.astype(np.float32)
      gws = o.gws.astype(np.float32)
      gb = (o.gb if o.gb is not None else np.zeros(O)).astype(np.float32)
      gwsum = o.gw.reshape(O, C).astype(np.float64).sum(axis=1).astype(np.float32)  # Σw per gemm out (vrmpy zp corr)
      blob_off = put(np.concatenate([[o.scc / 1.0, o.sr, 1.0 / o.s_bn, float(o.z_bn), o.s_bn], P, Q, gws, gb, gwsum]), np.float32)
      # Same mult-128 row-stride rule as se_gate above: head_gemm_range reads gw with dot_u8i8, whose vector
      # loads are ALIGNED, so a head with C%128 != 0 would have read row 1 onward from a misaligned address.
      # MobileNetV2 (1280) and distill (512) are both mult-128 by luck, which is why it has never fired.
      gwp = np.zeros((O, _ru(C, 128)), np.int8); gwp[:, :C] = o.gw.reshape(O, C)
      gw_off = put(gwp, np.int8)
      rec = pack("HD", op=OP_HEAD, out=out_off, conv=conv_off, res=res_off, gate=gate_off, blob=blob_off,
                 gw=gw_off, C=C, HW=HW, zcc=o.zcc, zr=o.zr, hasres=hasres, O=O, relu=int(o.relu),
                 has_gate=int(o.has_gate),
                 # gap[C] floats, then a second block reused twice (never live at once): head_gap_nhwc's
                 # mcp+acc (2 x C ints) and, after it, head_tail's qu[C] u8.
                 scratch=scr(C * 4) + (scr(C * 4) * 2 if not hd32 else scr(C)))
      if hd32:   # rec[14]=use head_d32, rec[15..17]=Ho,Wo,Wop
        rec[F("HD", "d32")], rec[F("HD", "H")], rec[F("HD", "W")], rec[F("HD", "Wop")] = 1, Hh, Ww, _ru(Ww, 4)
      op_outoff.append(out_off)
    recs.append(rec)
  out_name = ops[-1].out
  if seed_is_d32:   # PREPEND the entry pack: NHWC seed -> d32 seed. rec[1]=d32 out, rec[2]=nhwc in, rec[3..6]=W,H,Cigp,Wop
    sC, sH, sW = shp[seed_name][0]
    prec = pack("PK", op=OP_PACK, out=seed_d32_off, src=off[seed_name], W=sW, H=sH, Cigp=_ru(sC, 32), Wop=_ru(sW, 4))
    recs.insert(0, prec)
    op_outoff.insert(0, seed_d32_off)
  # The device arena is NHWC: a spatial [C,H,W] tensor is stored [H,W,C]. Offsets and sizes are layout-invariant,
  # so nothing above cares -- only a caller materialising the seed does (see reference.nhwc).
  return dict(
    ops=np.array(recs, np.int32),
    wts=bytes(blob),
    arena_size=cur,
    alloc_sizes=sizes,
    alloc_order=order,
    seed=(off[seed_name], shp[seed_name][0]),
    out=(off[out_name], int(np.prod(shp[out_name][0]))),
    out_f32=ops[-1].t == "HEAD",   # only the HEAD emits float; every other terminal (conv/dw/add/...) is u8

    op_outoff=op_outoff,
    op_list=ops,
  )
