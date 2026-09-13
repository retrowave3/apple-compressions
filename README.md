# Apple Compression Algorithms (27+)
![Tests](https://github.com/retrowave3/apple-compressions/actions/workflows/test.yml/badge.svg)


These implementations are based on iOS 27.0 (24A435). Each algorithm is kept independent as `.h` and `.c` pairs. Do not expect native libcompression performance. If you think something is wrong or missing open an issue.

- `lzraven`: selectors `0xD05` - testing shows config byte is currently ignored by libcompression.
- `lzmesh`: selectors `0xE00`, `0xE01`, `0xE05`, `0xE09`
- `lzbitmap`: selectors `0x600`-`0x602`, `0x700`-`0x702`

## Legal
 This project is not affiliated with, endorsed by, or sponsored by Apple Inc. Apple and macOS are trademarks of Apple Inc. All third-party trademarks and intellectual property remain the property of their respective owners.

## License
Licensed under the [MIT License](LICENSE).

The MIT License grants only rights that the contributors are authorized to license. It does not grant any rights on behalf of Apple or other third parties.