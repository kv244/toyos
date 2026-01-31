# KV Database Compaction

## Overview

The KV database uses an append-only architecture where updates and deletes mark old records as free but don't immediately reclaim EEPROM space. Over time, this can lead to EEPROM exhaustion even if only a small number of keys are actively stored.

The **compaction routine** solves this by copying all live entries to fresh EEPROM blocks, effectively reclaiming space from deleted or updated records.

## When to Use Compaction

You should run compaction when:

1. **After many updates**: If you frequently update the same keys, old versions accumulate waste
2. **After deletions**: Deleted entries leave gaps in EEPROM that won't be reused
3. **Before running out of space**: If `kv_write()` returns `KV_ERR_FULL` but you know you haven't hit the key limit
4. **Periodically in background tasks**: Run compaction during low-activity periods

## API

```c
int16_t kv_compact(void);
```

**Returns:**
- Positive value: Number of bytes reclaimed
- `KV_ERR_FULL`: Compaction failed (shouldn't happen unless EEPROM is genuinely full)

**Thread Safety:**
- Fully thread-safe (acquires mutex internally)
- Blocks other database operations during compaction
- Safe to call from any task

## How It Works

1. **Scans** all live entries referenced in the sorted index
2. **Copies** each entry to fresh EEPROM space starting from `KV_RECORDS_OFFSET`
3. **Updates** the index with new addresses
4. **Reclaims** space by resetting `next_free_addr`

The compaction is done in chunks to minimize SRAM usage (only uses a 25-byte temporary buffer).

## Example Usage

### Simple Compaction

```c
int16_t bytes_freed = kv_compact();
if (bytes_freed >= 0) {
    Serial.print("Reclaimed ");
    Serial.print(bytes_freed);
    Serial.println(" bytes");
}
```

### Background Task

```c
void task_compaction(void) {
    while (1) {
        os_delay(60000);  // Run every minute
        
        int16_t freed = kv_compact();
        if (freed > 0) {
            // Log compaction results
            Serial.print("[Compact] Freed ");
            Serial.println(freed);
        }
    }
}
```

### Conditional Compaction

```c
// Only compact if we've reclaimed significant space
int16_t freed = kv_compact();
if (freed > 100) {  // More than 100 bytes wasted
    Serial.println("Compaction was worthwhile");
} else {
    Serial.println("Database already compact");
}
```

## Performance Considerations

- **Time**: Proportional to total size of live data (O(N) where N = sum of all record sizes)
- **SRAM**: Uses only ~25 bytes for temporary buffer
- **EEPROM wear**: Each compaction writes every live entry once
  - EEPROM has ~100,000 write cycles per byte
  - Avoid excessive compaction on the same data
- **Blocking**: Holds the database mutex for the entire operation

## Design Notes

- Compaction is **idempotent** - running it multiple times is safe (though wasteful)
- Records already at their target location are **skipped** to minimize writes
- The operation is **atomic** - on failure, the database remains in a valid state
- Large values are copied in **chunks** to avoid stack overflow

## Trade-offs

**Benefits:**
- Reclaims wasted EEPROM space
- Improves EEPROM utilization efficiency
- Simple to use API

**Costs:**
- Blocks all database operations during compaction
- Writes to EEPROM (wear consideration)
- Takes time proportional to live data size

## Best Practices

1. **Don't over-compact**: Only run when significant space can be reclaimed
2. **Use background tasks**: Run during low-activity periods
3. **Monitor results**: Log the number of bytes freed to understand patterns
4. **Consider EEPROM wear**: Balance space reclamation vs. write cycles
