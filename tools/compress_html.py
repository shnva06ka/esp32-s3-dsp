import gzip

with open("test/index.html", "rb") as f:
    data = f.read()

gz = gzip.compress(data, compresslevel=9)

with open("src/eq_html.h", "w") as f:
    f.write("// Auto-generated from test/index.html\n")
    f.write("#pragma once\n")
    f.write("#include <Arduino.h>\n\n")
    f.write(f"const uint32_t PAGE_INDEX_HTML_GZ_LEN = {len(gz)};\n")
    f.write("const uint8_t PAGE_INDEX_HTML_GZ[] PROGMEM = {\n")
    for i in range(0, len(gz), 16):
        chunk = gz[i:i+16]
        hex_str = ", ".join(f"0x{b:02X}" for b in chunk)
        if i + 16 < len(gz):
            f.write(f"  {hex_str},\n")
        else:
            f.write(f"  {hex_str}\n")
    f.write("};\n")

print(f"Compressed {len(data)} bytes to {len(gz)} bytes in src/eq_html.h")

