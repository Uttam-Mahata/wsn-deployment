# Log File Splitter

This program splits the `loglistener.txt` file into multiple smaller files of 1000 lines each.

## Features

- Splits large log files into manageable chunks
- Creates up to 4 output files (4000 lines total)
- Preserves original formatting
- Provides progress feedback and summary

## Compilation

### Using the provided Makefile:
```bash
make -f Makefile.splitter
```

### Manual compilation:
```bash
gcc -Wall -Wextra -std=c99 -o log_splitter log_splitter.c
```

## Usage

1. Make sure `loglistener.txt` exists in the current directory
2. Run the compiled program:
```bash
./log_splitter
```

## Output Files

The program creates the following files:
- `loglistener_part_1.txt` (lines 1-1000)
- `loglistener_part_2.txt` (lines 1001-2000)
- `loglistener_part_3.txt` (lines 2001-3000)
- `loglistener_part_4.txt` (lines 3001-4000)

## Example Output

```
Created: loglistener_part_1.txt
Completed: loglistener_part_1.txt (1000 lines)
Created: loglistener_part_2.txt
Completed: loglistener_part_2.txt (1000 lines)
Created: loglistener_part_3.txt
Completed: loglistener_part_3.txt (1000 lines)
Created: loglistener_part_4.txt
Completed: loglistener_part_4.txt (845 lines)

=== SPLITTING SUMMARY ===
Total lines processed: 3845
Number of output files: 4
Files created:
  - loglistener_part_1.txt
  - loglistener_part_2.txt
  - loglistener_part_3.txt
  - loglistener_part_4.txt
```

## Error Handling

- Checks if input file exists
- Handles file creation errors
- Provides informative error messages

## Cleanup

To remove all generated files:
```bash
make -f Makefile.splitter clean
```

This will remove:
- The compiled `log_splitter` executable
- All `loglistener_part_*.txt` files
