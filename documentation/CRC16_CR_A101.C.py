import crcmod
from tkinter import filedialog

# Definiere den CRC-Typ (z.B. CRC-16-CCITT-FALSE)
# '0x1021' ist das Polynom, '0xFFFF' der Initialwert, 'True' das XOR-Ergebnis nachher
crc16_func = crcmod.mkCrcFun(0x11021, initCrc=0x0000, rev=False, xorOut=0x0000)

file_path = filedialog.askopenfilename()
chunk_size = 4096 # Leseblockgröße für große Dateien

crc_value = 0 # Initialwert für den CRC
with open(file_path, 'rb') as f:
    while True:
        chunk = f.read(chunk_size)
        if not chunk:
            break
        # Berechne den CRC für den aktuellen Chunk und update den Gesamtwert
        crc_value = crc16_func(chunk, crc_value)

print(f"CRC16-Wert für {file_path}: {crc_value:04X}") # Ausgabe als 4-stellige Hexadezimalzahl




# Angenommen, dies ist Ihre Hex-Daten-Zeichenkette
hex_string_data = "42420200400000000000000000000000" 
filling_zeros = "0000000000000000000000000000"

# Konvertiere die gesamte Hex-Zeichenkette in ein Byte-Objekt
# Die Methode bytes.fromhex() ist hierfür ideal

out_file = filedialog.asksaveasfile(mode="wb", defaultextension='.bin')
#'C:/temp/reveng/output.bin'
# Öffne die Binärdatei im Schreibmodus
#with open(out_file, 'wb') as f:
    # Schreibe die konvertierten Binärdaten
binary_data = bytes.fromhex(hex_string_data)
out_file.write(binary_data)
out_file.write(crc_value.to_bytes(2, byteorder='big'))
binary_data = bytes.fromhex(filling_zeros)
out_file.write(binary_data)

with open(file_path, 'rb') as source_file:
    binary_data = source_file.read()

# Writing the data of the previous file in the original file
# with open(out_file, 'ab') as destination_file:
out_file.write(binary_data)
    

