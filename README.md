# Apple Compression Algorithms (27+)
![Tests](https://github.com/retrowave3/apple-compression/actions/workflows/test.yml/badge.svg)


These implementations are based on black box reverse-engineering `libcompression.dylib` from iOS 27.0 (24A435). Each algorithm is kept independent as `.h` and `.c` pairs. Do not expect native `libcompression` performance. If you think something is wrong or missing open an issue.

- `lzraven.c` / `lzraven.h`: selectors `0xD05` - testing shows config byte is currently ignored by libcompression.
- `lzmesh.c` / `lzmesh.h`: selectors `0xE00`, `0xE01`, `0xE05`, `0xE09`
- `lzbitmap.c` / `lzbitmap.h`: selectors `0x600`-`0x602`, `0x700`-`0x702`

## Legal
 This project is not affiliated with, endorsed by, or sponsored by Apple Inc. Apple and macOS are trademarks of Apple Inc. All third-party trademarks and intellectual property remain the property of their respective owners.