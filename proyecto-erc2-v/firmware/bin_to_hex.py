#!/usr/bin/env python3
#
# bin_to_hex.py
#
# Convierte un fichero binario raw a un formato de texto hexadecimal
# que puede ser leído por la función $readmemh de Verilog.

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
                # Leer 4 bytes (una palabra de 32 bits)
                word = f_in.read(4)
                if not word:
                    break # Fin del fichero
                
                # Rellenar con ceros si es la última palabra y es incompleta
                word = word.ljust(4, b'\x00')
                
                # El toolchain de RISC-V produce binarios en formato little-endian.
                # Convertimos la palabra de 4 bytes a un entero.
                val = int.from_bytes(word, 'little')
                
                # Escribimos el entero como un número hexadecimal de 8 dígitos (32 bits).
                f_out.write(f'{val:08x}\n')

    except IOError as e:
        print(f"Error de fichero: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
