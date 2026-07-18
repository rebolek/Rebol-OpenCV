---
name: rebol-opencv-style
description: Idiomatic Rebol coding style for the Rebol-OpenCV extension's Rebol sources (examples.r3, test.r3, src/opencv-rebol-extension.r3, and any other *.r3 in this repo). Use whenever writing or editing Rebol code here — prefer iteration constructs (foreach/forall/repeat/loop/map-each) over manual while loops, build strings with ajoin/repend instead of chained append calls, use char! literals over single-character strings (no allocation), and match structured text with parse instead of hand-rolled scanning.
tools: Read, Edit, Grep, Bash
---

# Rebol Coding Style (Rebol-OpenCV)

This repo wraps OpenCV as a Rebol3 extension. The Rebol-side sources are
`examples.r3` (generates `README.md` from example blocks), `test.r3` (the
test suite), and `src/opencv-rebol-extension.r3` (the generated command/type
bindings plus the codegen script that produces them and the matching C code).
Apply this style to every `.r3` edit in this repo.

Target build is `Rebol/Bulk 3.22+` (per README, extension needs 3.14.0+; the
local `rebol3` binary used for testing this repo is 3.22.0). Verified directly
against that binary: `ajoin`, `repend`, `combine` (has `/with` for a
separator; **no** `delimit`), `collect`, `map-each`, `forall`, `repeat`,
`loop` are all present and behave as described below.

The guiding rule: **express intent with the right loop/series word, not with
manual index bookkeeping.** Reach for `while` only when no other iterator fits.

## 1. Prefer real iterators over `while`

`while` with a hand-managed counter or cursor is almost always the wrong tool.
Pick the iterator that matches the shape of the loop:

| You are doing… | Use | Not |
|---|---|---|
| iterating items of a series | `foreach x series [...]` | `while [...] [... series/i ...]` |
| walking a series by cursor (need the position) | `forall s [... s/1 ...]` | `while [not tail? s] [... s: next s]` |
| a fixed number of times | `loop n [...]` | `i: 0 while [i < n] [... i: i + 1]` |
| counting with an index | `repeat i n [...]` | manual counter |
| transforming items into a new block | `map-each x series [...]` | `append`-in-a-`while` |
| building a list conditionally | `collect [... keep x ...]` | `append` accumulator in `while` |
| paired/strided iteration | `foreach [k v] series [...]` | index arithmetic |

This repo already leans on paired `foreach` for spec tables — follow the same
shape for new spec-walking code (`src/opencv-rebol-extension.r3`, codegen
section):

```rebol
; Already idiomatic in this repo — match this shape for new code
foreach [name spec] handles [
    append handles-doc ajoin [LF LF "#### __" uppercase form name "__ - " spec/1 LF]
    foreach [name gets sets desc] next spec [
        append arg-words name
    ]
]
```

and `forall` when the loop needs the running position, as in `examples.r3`'s
live-camera example:

```rebol
forall data [data/1: random 255]
```

`while` is legitimate only for genuine condition-driven loops with no natural
series/count (e.g. `forever [... if 0 < waitKey 10 [break]]`-style event
loops). When you do use it, say why in a short comment.

## 2. Build strings with `ajoin` / `repend` / `combine`, not chained `append`

`ajoin` reduces its block, `form`s each value, and concatenates into one
fresh string:

```rebol
combine/with c-params ", "          ; "a, b, c"
```

`combine` joins a block of strings; `combine/with sep` inserts a separator
(this build has no `delimit`).

For **appending a reduced fragment into an existing buffer**, prefer `repend`
over `append buf ajoin [...]` — they produce identical output, but `repend`
skips the intermediate string `ajoin` would allocate:

```rebol
; Existing pattern throughout the codegen script in
; src/opencv-rebol-extension.r3 — works, but allocates an extra string:
append enu-commands ajoin ["^/^-CMD_OPENCV_" uppercase copy name #","]

; Prefer for new code — same result, one fewer allocation:
repend enu-commands ["^/^-CMD_OPENCV_" uppercase copy name #","]
```

Never use `append reduce [...]` — `repend` is the same operation with no
extra reduce/form step to get wrong.

Quick guide:
- accumulating into an existing buffer → `repend buffer [...]`
- producing a fresh joined string → `ajoin [...]` (mixed values) or `combine` (strings)
- need a separator between items → `combine/with items sep`

Single characters are `char!` literals, not one-character strings — the
codegen script already does this correctly (`#"-"`, `#"?"`, `#","`, `#"^""` in
`src/opencv-rebol-extension.r3`); keep matching that in new code rather than
writing `"-"`, `"?"`, etc. `append`/`repend`/`ajoin` elements, `replace`
targets, and `find`/`parse` matches all accept `char!` directly:

```rebol
; Wrong — allocates a one-char string each time
replace/all word "-" "_"

; Right — char! is a scalar, no allocation (matches existing repo style)
replace/all word #"-" #"_"
```

> Note: `append/only` still applies when you mean to append a *block as one
> item* rather than flatten it. `ajoin`/`repend` are for flattening; don't use
> them where you need the block structure preserved (e.g.
> `append/only reb-code mold spec` in the codegen script).

## 3. Match with `parse`, not imperative scanning

When recognizing or transforming structured text — the example blocks in
`examples.r3`, the command spec table in `src/opencv-rebol-extension.r3` —
use `parse` with a rule rather than indexing and advancing by hand. Both
files already do this for their generator logic:

```rebol
; examples.r3 — walking `comment`/`example` pairs to emit the README
parse examples [ any [
      'comment set str: string! (emit-comment str)
    | ['example set str: string! (emit-heading str)
                set str: string! (emit-code str)]
    | skip
]]
```

Match that shape for new spec-walking or text-scanning code instead of a
`while`/`if` ladder.

## 4. Other idioms

- Use `unless cond [...]` instead of `if not cond [...]`; `either` for
  two-branch, not `if`/`if`. Both appear already in `test.r3`/`examples.r3`.
- Use `case`/`switch` over long `either` chains for new multi-branch logic
  (neither appears yet in this repo, but prefer them over an `either` chain
  when adding one).
- Use `any [...]` / `all [...]` for short-circuit logic and defaulting
  (`x: any [maybe-x default]`) rather than nested `if`.
- Use `select`/`find`/`pick` instead of looping to locate an element.
- `map-each` / `collect ... keep` instead of preallocating and pushing in a loop.

## 5. Document new functions

`test.r3`'s helpers (`header`, `assert`, `assert-equal`, `assert-error`) each
carry a docstring plus a description on every argument — follow that pattern
for any new hand-written `func`/`function` (test helpers, command wrappers).
Older generator code in `examples.r3`/`src/opencv-rebol-extension.r3`
predates this convention (e.g. `readme: func[val][...]`) — don't take that as
license to skip docs in new code, but don't rewrite untouched generator
helpers just to add docstrings either.

```rebol
assert-equal: func [
    "Fail if a and b are not equal."
    a "Expected value."
    b "Actual value."
    label [string!] "Test label."
][
    ...
]
```

## Self-check before finishing an edit

```sh
# while loops you introduced — justify or replace each
grep -n "while \[" <file>
# append reduce — should be repend
grep -n "append reduce" <file>
# append <buf> ajoin [...] — candidate for repend
grep -n "append .* ajoin \[" <file>
# single-character string literals — should usually be char! (#"x")
grep -nE '"[^"\\]"' <file>
```
