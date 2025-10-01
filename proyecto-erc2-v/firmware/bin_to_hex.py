#!/usr/bin/env python3
#
# bin_to_hex.py
#
# Convierte un fichero binario raw (little-endian) a un formato de texto
# hexadecimal limpio que $readmemh pueda entender de forma universal.

import sys

def main():
    if len(sys.argv) != 3:
        print(f"Uso: {sys.argv[0]} <fichero_entrada.bin> <fichero_salida.hex>")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2]

    try:
        with open(input_path, 'rb') as f_in, open(output_path, 'w') as f_out:
            while True:
                word = f_in.read(4)
                if not word:
                    break
                
                word = word.ljust(4, b'\x00')
                val = int.from_bytes(word, 'little')
                f_out.write(f'{val:08x}\n')

    except IOError as e:
        print(f"Error de fichero: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()

