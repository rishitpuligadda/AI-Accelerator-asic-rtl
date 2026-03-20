import numpy as np

# =============================================================================
# Shared helpers
# =============================================================================
def direct_conv(img, k, stride=1, pad=0):
    if pad > 0:
        img = np.pad(img, pad, mode='constant')
    img_h, img_w = img.shape
    k_h,   k_w   = k.shape
    out_h = (img_h - k_h) // stride + 1
    out_w = (img_w - k_w) // stride + 1
    output = np.zeros((out_h, out_w), dtype=np.int64)
    for r in range(out_h):
        for c in range(out_w):
            patch = img[r*stride:r*stride+k_h, c*stride:c*stride+k_w]
            output[r, c] = np.sum(patch * k)
    return output

def im2col(img, k_h, k_w, stride=1, pad=0):
    if pad > 0:
        img = np.pad(img, pad, mode='constant')
    img_h, img_w = img.shape
    out_h = (img_h - k_h) // stride + 1
    out_w = (img_w - k_w) // stride + 1
    col = np.zeros((out_h * out_w, k_h * k_w), dtype=np.int64)
    idx = 0
    for r in range(out_h):
        for c in range(out_w):
            col[idx] = img[r*stride:r*stride+k_h,
                           c*stride:c*stride+k_w].flatten()
            idx += 1
    return col

def print_header(title):
    print("\n" + "=" * 55)
    print(f"  {title}")
    print("=" * 55)

def verify(name, result, expected):
    if np.array_equal(result, expected):
        print(f"  PASS  {name}")
    else:
        print(f"  FAIL  {name}")
        print(f"    got:      {result.flatten()}")
        print(f"    expected: {expected.flatten()}")


# =============================================================================
# Shared 4x4 test image (used in all examples)
# =============================================================================
image = np.array([
    [ 1,  2,  3,  4],
    [ 5,  6,  7,  8],
    [ 9, 10, 11, 12],
    [13, 14, 15, 16]
], dtype=np.int32)


# =============================================================================
# Example 1 — Sobel-X (horizontal edge detection)
# =============================================================================
print_header("Example 1 — Sobel-X (horizontal edges)")

sobel_x = np.array([
    [ 1,  0, -1],
    [ 2,  0, -2],
    [ 1,  0, -1]
], dtype=np.int32)

r1 = direct_conv(image, sobel_x)
print(f"\n Kernel:\n{sobel_x}")
print(f"\n Output (2x2):\n{r1}")
# All -8 because every column pair differs by 2 uniformly
verify("Sobel-X all -8", r1, np.full((2,2), -8, dtype=np.int64))


# =============================================================================
# Example 2 — Sobel-Y (vertical edge detection)
# =============================================================================
print_header("Example 2 — Sobel-Y (vertical edges)")

sobel_y = np.array([
    [ 1,  2,  1],
    [ 0,  0,  0],
    [-1, -2, -1]
], dtype=np.int32)

r2 = direct_conv(image, sobel_y)
print(f"\n Kernel:\n{sobel_y}")
print(f"\n Output (2x2):\n{r2}")

# Hand-verify patch(0,0):
# dot([1,2,3,5,6,7,9,10,11], [1,2,1,0,0,0,-1,-2,-1])
# = 1+4+3 + 0 + -9-20-11 = 8 - 40 = -32
exp2 = np.array([[-32, -32], [-32, -32]], dtype=np.int64)
verify("Sobel-Y all -32", r2, exp2)


# =============================================================================
# Example 3 — Laplacian (edge detection in all directions)
# =============================================================================
print_header("Example 3 — Laplacian (all-direction edges)")

laplacian = np.array([
    [ 0,  1,  0],
    [ 1, -4,  1],
    [ 0,  1,  0]
], dtype=np.int32)

r3 = direct_conv(image, laplacian)
print(f"\n Kernel:\n{laplacian}")
print(f"\n Output (2x2):\n{r3}")

# Hand-verify patch(0,0):
# dot([1,2,3,5,6,7,9,10,11], [0,1,0,1,-4,1,0,1,0])
# = 0+2+0 + 5-24+7 + 0+10+0 = 0
# Uniform gradient image -> Laplacian = 0 everywhere (flat 2nd derivative)
exp3 = np.zeros((2,2), dtype=np.int64)
verify("Laplacian all 0", r3, exp3)


# =============================================================================
# Example 4 — Box blur (3x3 average filter)
# =============================================================================
print_header("Example 4 — Box blur (3x3 average)")

box_blur = np.array([
    [1, 1, 1],
    [1, 1, 1],
    [1, 1, 1]
], dtype=np.int32)

r4 = direct_conv(image, box_blur)
print(f"\n Kernel (sum of 3x3 patch — divide by 9 for true average):\n{box_blur}")
print(f"\n Output (2x2) — raw sums:\n{r4}")
print(f"\n Output (2x2) — divided by 9 (true average):")
print(np.round(r4 / 9, 2))

# Hand-verify patch(0,0): sum(1..3, 5..7, 9..11) = 54
exp4 = np.array([[54, 63], [90, 99]], dtype=np.int64)
verify("Box blur sums", r4, exp4)


# =============================================================================
# Example 5 — Sharpen filter
# =============================================================================
print_header("Example 5 — Sharpen filter")

sharpen = np.array([
    [ 0, -1,  0],
    [-1,  5, -1],
    [ 0, -1,  0]
], dtype=np.int32)

r5 = direct_conv(image, sharpen)
print(f"\n Kernel:\n{sharpen}")
print(f"\n Output (2x2):\n{r5}")

# The image is a 1..16 ramp so each output pixel is different.
# Hand-verify each patch dot product:
#   patch(0,0): [1,2,3,5,6,7,9,10,11]  -> 0-2+0-5+30-7+0-10+0  =  6
#   patch(0,1): [2,3,4,6,7,8,10,11,12] -> 0-3+0-6+35-8+0-11+0  =  7
#   patch(1,0): [5,6,7,9,10,11,13,14,15]-> 0-6+0-9+50-11+0-14+0 = 10
#   patch(1,1): [6,7,8,10,11,12,14,15,16]->0-7+0-10+55-12+0-15+0= 11
# NOTE: expected values are NOT all 6 — the original assertion was wrong.
exp5 = np.array([[6, 7], [10, 11]], dtype=np.int64)
verify("Sharpen [6,7,10,11]", r5, exp5)


# =============================================================================
# Example 6a — Stride=2 (subsampled convolution)
# =============================================================================
print_header("Example 6a — Stride=2 (no padding)")

# Use a larger 6x6 image so stride=2 gives a meaningful output
image_6x6 = np.array([
    [ 1,  2,  3,  4,  5,  6],
    [ 7,  8,  9, 10, 11, 12],
    [13, 14, 15, 16, 17, 18],
    [19, 20, 21, 22, 23, 24],
    [25, 26, 27, 28, 29, 30],
    [31, 32, 33, 34, 35, 36]
], dtype=np.int32)

r6a = direct_conv(image_6x6, sobel_x, stride=2)
out_h = (6 - 3) // 2 + 1
out_w = (6 - 3) // 2 + 1
print(f"\n Image: 6x6,  Kernel: 3x3,  Stride: 2")
print(f" Output size: ({out_h} x {out_w})")
print(f"\n Output:\n{r6a}")

# im2col with stride — verify both methods match
col6a = im2col(image_6x6, 3, 3, stride=2)
r6a_im2col = (col6a @ sobel_x.flatten().reshape(-1,1)).reshape(out_h, out_w)
verify("Stride=2 direct==im2col", r6a, r6a_im2col)


# =============================================================================
# Example 6b — Same-padding (output same size as input)
# =============================================================================
print_header("Example 6b — Same padding (pad=1, stride=1)")

# With pad=1 and stride=1, output size = input size = 4x4
r6b = direct_conv(image, sobel_x, stride=1, pad=1)
print(f"\n Image: 4x4,  Kernel: 3x3,  Pad: 1,  Stride: 1")
print(f" Output size: {r6b.shape}  (same as input)")
print(f"\n Output (4x4):\n{r6b}")

# im2col with padding — verify both methods match
col6b      = im2col(image, 3, 3, stride=1, pad=1)
r6b_im2col = (col6b @ sobel_x.flatten().reshape(-1,1)).reshape(4,4)
verify("Same-pad direct==im2col", r6b, r6b_im2col)


# =============================================================================
# Summary
# =============================================================================
print_header("Summary of all outputs")
print(f"\n Ex1  Sobel-X   (2x2): {r1.flatten()}")
print(f" Ex2  Sobel-Y   (2x2): {r2.flatten()}")
print(f" Ex3  Laplacian (2x2): {r3.flatten()}")
print(f" Ex4  Box blur  (2x2): {r4.flatten()}  (divide by 9 for avg)")
print(f" Ex5  Sharpen   (2x2): {r5.flatten()}  (ramp image -> not uniform)")
print(f" Ex6a Stride=2  (2x2): {r6a.flatten()}")
print(f" Ex6b Same-pad  (4x4):\n{r6b}")