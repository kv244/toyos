import sys
import re

def parse_map_file(map_file_path):
    print(f"Analyzing {map_file_path} for RAM usage...")
    
    # Symbols in .data and .bss
    symbol_regex = re.compile(r'^\s*0x[0-9a-f]+\s+(0x[0-9a-f]+)\s+(.+)$')
    # Section headers to identify when we are in .data or .bss (approximate for typical AVR map)
    
    symbols = []
    
    current_section = None
    
    try:
        with open(map_file_path, 'r') as f:
            lines = f.readlines()
            
        for line in lines:
            line = line.strip()
            
            # Detect section
            if line.startswith('.data'):
                current_section = 'data'
            elif line.startswith('.bss'):
                current_section = 'bss'
            elif line.startswith('.text'):
                current_section = 'text' # Flash, ignore for RAM
            
            if current_section in ['data', 'bss']:
                # Typical line: 0x00800100       0x14 objects/foo.o
                # or:           0x00800100                global_var
                # The format varies. We look for lines with an address, size, and name.
                # GCC Map file usually has:
                # .bss.varname   0x00800123       0x4 foo.o
                
                parts = line.split()
                if len(parts) >= 3 and parts[1].startswith('0x'):
                     # .data.foo   0x00800100      0x10 foo.o
                     # Or just:
                     #  COMMON         0x00800200       0x4 foo.o
                     try:
                         size = int(parts[1], 16)
                         if size > 0:
                             name = parts[0]
                             if name.startswith('.'):
                                 name = name[1:] # strip leading dot
                             
                             # If file info is present
                             file_info = parts[2] if len(parts) > 2 else "?"
                             
                             symbols.append({
                                 'name': name,
                                 'size': size,
                                 'section': current_section,
                                 'file': file_info
                             })
                     except ValueError:
                         pass

        # Sort by size descending
        symbols.sort(key=lambda x: x['size'], reverse=True)
        
        print(f"{'SIZE (Bytes)':<12} {'SECTION':<8} {'NAME':<40} {'FILE'}")
        print("-" * 80)
        
        total_ram = 0
        for s in symbols[:20]: # Top 20
            print(f"{s['size']:<12} {s['section']:<8} {s['name']:<40} {s['file']}")
            total_ram += s['size']
            
        print("-" * 80)
        print(f"Total tracked in top symbols: {total_ram} bytes")
        
    except FileNotFoundError:
        print("Error: Map file not found.")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_map.py <path_to_map_file>")
        sys.exit(1)
    parse_map_file(sys.argv[1])
