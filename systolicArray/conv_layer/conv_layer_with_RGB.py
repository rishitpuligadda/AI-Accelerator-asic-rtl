import numpy as np

# =============================================================================
# Shared helpers
# =============================================================================
def direct_conv_multichannel(img, k, stride=1, pad=0):
    """
    img: [H, W, C_in]
    k:   [K_h, K_w, C_in]
    returns: [out_h, out_w]  (single output channel)
    """
    if pad > 0:
        img = np.pad(img, ((pad,pad),(pad,pad),(0,0)), mode='constant')
    img_h, img_w, c_in = img.shape
    k_h,   k_w,   _    = k.shape
    out_h = (img_h - k_h) // stride + 1
    out_w = (img_w - k_w) // stride + 1
    output = np.zeros((out_h, out_w), dtype=np.int64)
    for r in range(out_h):
        for c in range(out_w):
            patch = img[r*stride:r*stride+k_h, c*stride:c*stride+k_w, :]
            output[r, c] = np.sum(patch * k)
    return output

def im2col_multichannel(img, k_h, k_w, stride=1, pad=0):
    """
    img: [H, W, C_in]
    returns: [out_h*out_w, k_h*k_w*C_in]
    Flattening order: for each spatial position, all channels interleaved.
    Matches the SV col_matrix row layout:
      [row0_ch0, row0_ch1, row0_ch2, row1_ch0, ...] per kernel position
    """
    if pad > 0:
        img = np.pad(img, ((pad,pad),(pad,pad),(0,0)), mode='constant')
    img_h, img_w, c_in = img.shape
    out_h = (img_h - k_h) // stride + 1
    out_w = (img_w - k_w) // stride + 1
    col = np.zeros((out_h * out_w, k_h * k_w * c_in), dtype=np.int64)
    idx = 0
    for r in range(out_h):
        for c in range(out_w):
            patch = img[r*stride:r*stride+k_h, c*stride:c*stride+k_w, :]
            # flatten: spatial first, then channel — matches SV kern_flat layout
            col[idx] = patch.reshape(k_h * k_w, c_in).flatten()
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
# Shared 4x4x3 RGB test image
# R channel: 1..16  (same as single-channel tests)
# G channel: 17..32
# B channel: 33..48
# =============================================================================
R = np.arange(1,  17, dtype=np.int32).reshape(4,4)
G = np.arange(17, 33, dtype=np.int32).reshape(4,4)
B = np.arange(33, 49, dtype=np.int32).reshape(4,4)
image_rgb = np.stack([R, G, B], axis=-1)   # shape [4,4,3]

print_header("RGB test image (4x4x3)")
print(f"\n R channel:\n{R}")
print(f"\n G channel:\n{G}")
print(f"\n B channel:\n{B}")


# =============================================================================
# Example 1 — Sobel-X applied to all 3 channels equally
# Kernel: same Sobel-X for each channel
# Expected: sum of per-channel Sobel-X responses
# =============================================================================
print_header("Example 1 — Sobel-X (RGB, equal weights)")

sobel_x_2d = np.array([
    [ 1,  0, -1],
    [ 2,  0, -2],
    [ 1,  0, -1]
], dtype=np.int32)

# Same kernel applied to all 3 channels — stack along channel axis
sobel_x_3ch = np.stack([sobel_x_2d, sobel_x_2d, sobel_x_2d], axis=-1)  # [3,3,3]

r1 = direct_conv_multichannel(image_rgb, sobel_x_3ch)
print(f"\n Kernel (same for all channels):\n{sobel_x_2d}")
print(f"\n Output (2x2):\n{r1}")

# Verify with im2col
col1     = im2col_multichannel(image_rgb, 3, 3)
kern1_flat = sobel_x_3ch.reshape(3*3, 3).flatten()   # [k_h*k_w*C_in]
r1_im2col  = (col1 @ kern1_flat.reshape(-1,1)).reshape(2,2)
verify("Sobel-X RGB direct==im2col", r1, r1_im2col)

# Manual check: R contributes -8, G contributes -8, B contributes -8 -> total -24
r1_R = np.array([[ 1, 2, 3, 5, 6, 7, 9,10,11]], dtype=np.int64)
dot_R = int((r1_R @ sobel_x_2d.flatten().reshape(-1,1)).flatten()[0])
print(f"\n R-only patch(0,0) dot Sobel-X = {dot_R}  (expect -8)")
print(f" Total patch(0,0) across 3 channels = {r1[0,0]}  (expect -24)")
exp1 = np.full((2,2), -24, dtype=np.int64)
verify("Sobel-X RGB all -24", r1, exp1)


# =============================================================================
# Example 2 — Sobel-Y applied to all 3 channels equally
# =============================================================================
print_header("Example 2 — Sobel-Y (RGB, equal weights)")

sobel_y_2d = np.array([
    [ 1,  2,  1],
    [ 0,  0,  0],
    [-1, -2, -1]
], dtype=np.int32)

sobel_y_3ch = np.stack([sobel_y_2d, sobel_y_2d, sobel_y_2d], axis=-1)

r2 = direct_conv_multichannel(image_rgb, sobel_y_3ch)
print(f"\n Kernel (same for all channels):\n{sobel_y_2d}")
print(f"\n Output (2x2):\n{r2}")

col2      = im2col_multichannel(image_rgb, 3, 3)
kern2_flat = sobel_y_3ch.reshape(3*3, 3).flatten()
r2_im2col  = (col2 @ kern2_flat.reshape(-1,1)).reshape(2,2)
verify("Sobel-Y RGB direct==im2col", r2, r2_im2col)

# R contributes -32, G -32, B -32 -> total -96
exp2 = np.full((2,2), -96, dtype=np.int64)
verify("Sobel-Y RGB all -96", r2, exp2)


# =============================================================================
# Example 3 — Laplacian on RGB
# =============================================================================
print_header("Example 3 — Laplacian (RGB, equal weights)")

laplacian_2d = np.array([
    [ 0,  1,  0],
    [ 1, -4,  1],
    [ 0,  1,  0]
], dtype=np.int32)

laplacian_3ch = np.stack([laplacian_2d, laplacian_2d, laplacian_2d], axis=-1)

r3 = direct_conv_multichannel(image_rgb, laplacian_3ch)
print(f"\n Kernel (same for all channels):\n{laplacian_2d}")
print(f"\n Output (2x2):\n{r3}")

col3       = im2col_multichannel(image_rgb, 3, 3)
kern3_flat = laplacian_3ch.reshape(3*3, 3).flatten()
r3_im2col  = (col3 @ kern3_flat.reshape(-1,1)).reshape(2,2)
verify("Laplacian RGB direct==im2col", r3, r3_im2col)

# Each channel is a uniform ramp so Laplacian = 0 per channel -> total 0
exp3 = np.zeros((2,2), dtype=np.int64)
verify("Laplacian RGB all 0", r3, exp3)


# =============================================================================
# Example 4 — Box blur on RGB
# =============================================================================
print_header("Example 4 — Box blur (RGB, equal weights)")

box_blur_2d = np.ones((3,3), dtype=np.int32)
box_blur_3ch = np.stack([box_blur_2d, box_blur_2d, box_blur_2d], axis=-1)

r4 = direct_conv_multichannel(image_rgb, box_blur_3ch)
print(f"\n Kernel: all-ones 3x3 per channel")
print(f"\n Output (2x2) — raw sums across all 3 channels:\n{r4}")

col4       = im2col_multichannel(image_rgb, 3, 3)
kern4_flat = box_blur_3ch.reshape(3*3, 3).flatten()
r4_im2col  = (col4 @ kern4_flat.reshape(-1,1)).reshape(2,2)
verify("Box blur RGB direct==im2col", r4, r4_im2col)

# R: 54,63,90,99   G: each +9*16=144 offset -> 54+144=198 etc   B: +9*32=288
# patch(0,0): R=54, G=54+9*16=198, B=54+9*32=342 -> total=594
exp4 = np.array([[594, 621], [702, 729]], dtype=np.int64)
verify("Box blur RGB sums", r4, exp4)


# =============================================================================
# Example 5 — Sharpen on RGB
# =============================================================================
print_header("Example 5 — Sharpen (RGB, equal weights)")

sharpen_2d = np.array([
    [ 0, -1,  0],
    [-1,  5, -1],
    [ 0, -1,  0]
], dtype=np.int32)

sharpen_3ch = np.stack([sharpen_2d, sharpen_2d, sharpen_2d], axis=-1)

r5 = direct_conv_multichannel(image_rgb, sharpen_3ch)
print(f"\n Kernel (same for all channels):\n{sharpen_2d}")
print(f"\n Output (2x2):\n{r5}")

col5       = im2col_multichannel(image_rgb, 3, 3)
kern5_flat = sharpen_3ch.reshape(3*3, 3).flatten()
r5_im2col  = (col5 @ kern5_flat.reshape(-1,1)).reshape(2,2)
verify("Sharpen RGB direct==im2col", r5, r5_im2col)

# R: [6,7,10,11]  G and B channels each add a constant offset
# G patch(0,0): [17,18,19,21,22,23,25,26,27]
# dot sharpen: 0-18+0-21+110-23+0-26+0 = 22
# B patch(0,0): [33,34,35,37,38,39,41,42,43]
# dot sharpen: 0-34+0-37+190-39+0-42+0 = 38
# total(0,0) = 6+22+38 = 66
exp5 = np.array([[66, 69], [78, 81]], dtype=np.int64)
verify("Sharpen RGB", r5, exp5)


# =============================================================================
# Example 6 — Same padding (pad=1, Sobel-X, RGB)
# =============================================================================
print_header("Example 6 — Same padding (pad=1, Sobel-X, RGB)")

r6 = direct_conv_multichannel(image_rgb, sobel_x_3ch, stride=1, pad=1)
print(f"\n Image: 4x4x3,  Kernel: 3x3x3,  Pad: 1,  Stride: 1")
print(f" Output size: {r6.shape}  (same spatial size as input)")
print(f"\n Output (4x4):\n{r6}")

col6       = im2col_multichannel(image_rgb, 3, 3, stride=1, pad=1)
kern6_flat = sobel_x_3ch.reshape(3*3, 3).flatten()
r6_im2col  = (col6 @ kern6_flat.reshape(-1,1)).reshape(4,4)
verify("Same-pad RGB direct==im2col", r6, r6_im2col)


# =============================================================================
# Example 7 — Per-channel different kernels (luminance weighting)
# Simulates a real CNN where each channel has its own learned weight
# Weights approximate human luminance: R*0.299, G*0.587, B*0.114
# Using integer approximation: R*3, G*6, B*1
# =============================================================================
print_header("Example 7 — Per-channel kernels (luminance-weighted Sobel-X)")

sobel_x_R = sobel_x_2d * 3   # weight R channel
sobel_x_G = sobel_x_2d * 6   # weight G channel
sobel_x_B = sobel_x_2d * 1   # weight B channel
sobel_x_weighted = np.stack([sobel_x_R, sobel_x_G, sobel_x_B], axis=-1)

r7 = direct_conv_multichannel(image_rgb, sobel_x_weighted)
print(f"\n R kernel weight: 3,  G kernel weight: 6,  B kernel weight: 1")
print(f"\n Output (2x2):\n{r7}")

col7       = im2col_multichannel(image_rgb, 3, 3)
kern7_flat = sobel_x_weighted.reshape(3*3, 3).flatten()
r7_im2col  = (col7 @ kern7_flat.reshape(-1,1)).reshape(2,2)
verify("Luminance-weighted Sobel-X direct==im2col", r7, r7_im2col)

# R: -8*3=-24, G: -8*6=-48, B: -8*1=-8  -> total = -80
exp7 = np.full((2,2), -80, dtype=np.int64)
verify("Luminance-weighted all -80", r7, exp7)


# =============================================================================
# Summary
# =============================================================================
print_header("Summary — kernel flat layout for SV testbench")
print("""
  kern_flat layout for C_in=3 (interleaved by spatial position):
  index:  0      1      2      3      4      5   ...  26
          k[0,0,R] k[0,0,G] k[0,0,B] k[0,1,R] k[0,1,G] k[0,1,B] ... k[2,2,B]

  col_matrix row layout (one output pixel per row):
  index:  0      1      2      3      4      5   ...  26
          p[0,0,R] p[0,0,G] p[0,0,B] p[0,1,R] p[0,1,G] p[0,1,B] ... p[2,2,B]
""")
print(f" Ex1  Sobel-X RGB   (2x2): {r1.flatten()}")
print(f" Ex2  Sobel-Y RGB   (2x2): {r2.flatten()}")
print(f" Ex3  Laplacian RGB (2x2): {r3.flatten()}")
print(f" Ex4  Box blur RGB  (2x2): {r4.flatten()}")
print(f" Ex5  Sharpen RGB   (2x2): {r5.flatten()}")
print(f" Ex6  Same-pad RGB  (4x4):\n{r6}")
print(f" Ex7  Luma-Sobel-X  (2x2): {r7.flatten()}")