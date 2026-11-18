import crcmod
from tkinter import filedialog
custom_crc_table = [0] * 256  # Initialisierung der Tabelle
def generate_crc32_table(_poly):

    global custom_crc_table

    for i in range(256):
        c = i << 24

        for j in range(8):
            c = (c << 1) ^ _poly if (c & 0x80000000) else c << 1

        custom_crc_table[i] = c & 0xffffffff


def crc32_stm(bytes_arr):

    length = len(bytes_arr)
    crc = 0xffffffff

    k = 0
    while length >= 4:

        v = ((bytes_arr[k] << 24) & 0xFF000000) | ((bytes_arr[k+1] << 16) & 0xFF0000) | \
        ((bytes_arr[k+2] << 8) & 0xFF00) | (bytes_arr[k+3] & 0xFF)

        crc = ((crc << 8) & 0xffffffff) ^ custom_crc_table[0xFF & ((crc >> 24) ^ v)]
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc_table[0xFF & ((crc >> 24) ^ (v >> 8))]
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc_table[0xFF & ((crc >> 24) ^ (v >> 16))]
        crc = ((crc << 8) & 0xffffffff) ^ custom_crc_table[0xFF & ((crc >> 24) ^ (v >> 24))]

        k += 4
        length -= 4

    if length > 0:
        v = 0

        for i in range(length):
            v |= (bytes_arr[k+i] << 24-i*8)

        if length == 1:
            v &= 0xFF000000

        elif length == 2:
            v &= 0xFFFF0000

        elif length == 3:
            v &= 0xFFFFFF00

        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc_table[0xFF & ( (crc >> 24) ^ (v ) )];
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc_table[0xFF & ( (crc >> 24) ^ (v >> 8) )];
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc_table[0xFF & ( (crc >> 24) ^ (v >> 16) )];
        crc = (( crc << 8 ) & 0xffffffff) ^ custom_crc_table[0xFF & ( (crc >> 24) ^ (v >> 24) )];

    return crc

poly = 0x04C11DB7


generate_crc32_table(poly)
# Definiere den CRC-Typ (z.B. CRC-16-CCITT-FALSE)
# '0x1021' ist das Polynom, '0xFFFF' der Initialwert, 'True' das XOR-Ergebnis nachher
crc16_func = crcmod.mkCrcFun(0x11021, initCrc=0x0000, rev=False, xorOut=0x0000)

file_path = filedialog.askopenfilename()
chunk_size = 4096 # Leseblockgröße für große Dateien

crc_value = 0 # Initialwert für den CRC


with open(file_path, 'rb') as source_file:
    global original_data
    original_data = source_file.read()
    crc_stm = crc32_stm(bytearray(original_data))
    crc_bytes_le = crc_stm.to_bytes(4, byteorder='little')
    original_data = bytearray(original_data)  # in bytearray umwandeln
    original_data.extend(crc_bytes_le)        # crc_bytes_le anhängen
    for i in range(0, len(original_data), chunk_size):
        chunk = original_data[i:i+chunk_size]
        crc_value = crc16_func(chunk, crc_value)
    Bytes14and15=len(original_data)%65536
    source_file.close

print(f"CRC16-Wert für {file_path}: {crc_value:04X}") # Ausgabe als 4-stellige Hexadezimalzahl
print(f"CRC16-Wert für {file_path}: {Bytes14and15:04X}")


# Angenommen, dies ist Ihre Hex-Daten-Zeichenkette
hex_string_data = "0145824040000000000000000000" 
filling_zeros = "0000000000000000000000000000"

# Konvertiere die gesamte Hex-Zeichenkette in ein Byte-Objekt
# Die Methode bytes.fromhex() ist hierfür ideal

out_file = filedialog.asksaveasfile(mode="wb", defaultextension='.bin')
#'C:/temp/reveng/output.bin'
# Öffne die Binärdatei im Schreibmodus
#with open(out_file, 'wb') as f:
    # Schreibe die konvertierten Binärdaten
binary_data = bytes.fromhex(hex_string_data)
binary_data = bytearray(binary_data)
binary_data.extend(Bytes14and15.to_bytes(2, byteorder='big'))
binary_data.extend(crc_value.to_bytes(2, byteorder='big'))
binary_data.extend(bytes.fromhex(filling_zeros))
binary_data.extend(original_data)
#out_file.write(binary_data)
#out_file.write(Bytes14and15.to_bytes(2, byteorder='big'))
#out_file.write(crc_value.to_bytes(2, byteorder='big'))
#binary_data = bytes.fromhex(filling_zeros)
out_file.write(binary_data)

#with open(file_path, 'rb') as source_file:
#    binary_data = source_file.read()

# Writing the data of the previous file in the original file
# with open(out_file, 'ab') as destination_file:

#out_file.write(binary_data)

out_file.close()
