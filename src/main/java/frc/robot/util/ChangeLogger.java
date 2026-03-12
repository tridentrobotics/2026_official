package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

/**
 * Small utility that only prints a message for a given key when it changes
 * from the previous value. Use one per class (or shared) to reduce log spam.
 */
public class ChangeLogger {
    private final Map<String, String> lastLogs = new HashMap<>();
    private final String prefix;

    public ChangeLogger(String prefix) {
        this.prefix = (prefix == null || prefix.isEmpty()) ? "" : prefix + ": ";
    }

    public ChangeLogger() {
        this("");
    }

    public void logOnce(String key, String message) {
        String prev = lastLogs.get(key);
        if (prev == null || !prev.equals(message)) {
            System.out.println(prefix + message);
            lastLogs.put(key, message);
        }
    }

    public void clear() {
        lastLogs.clear();
    }
}
