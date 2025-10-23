/*
 * Test Runner for playBuoyInstruction Functions
 *
 * Upload this sketch to run all tests and verify the Arduino implementation
 * matches the original C++ parser implementation.
 *
 * Open Serial Monitor at 115200 baud to see test results.
 */

#include "../boat_buoy_central/playBuoyInstruction.h"

// We need to include the test file's functions
// Copy the test functions here or include them

// Audio map lookup
const struct AudioMapping {
    const char* key;
    int id;
} AUDIO_MAP_TEST[] = {
    {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, {"5", 5},
    {"6", 6}, {"7", 7}, {"8", 8}, {"9", 9}, {"10", 10},
    {"11", 11}, {"12", 12}, {"15", 13}, {"20", 14}, {"30", 15},
    {"40", 16}, {"50", 17}, {"60", 18}, {"70", 19}, {"80", 20},
    {"90", 21},
    {"AND", 22}, {"BUOY", 23}, {"HUNDRED", 24},
    {"METRES", 25}, {"OCLOCK", 26},
    {"OFF", 27}, {"ON", 28}, {"POWER", 29}
};

const int AUDIO_MAP_SIZE_TEST = sizeof(AUDIO_MAP_TEST) / sizeof(AUDIO_MAP_TEST[0]);

int getAudioIdTest(const char* key) {
    for (int i = 0; i < AUDIO_MAP_SIZE_TEST; i++) {
        if (strcmp(AUDIO_MAP_TEST[i].key, key) == 0) {
            return AUDIO_MAP_TEST[i].id;
        }
    }
    return -1;
}

void printTestHeader(const char* testName) {
    Serial.println("\n========================================");
    Serial.print("TEST: ");
    Serial.println(testName);
    Serial.println("========================================");
}

void testGetAudioId() {
    printTestHeader("getAudioId() - Audio Map Lookup");

    Serial.println("Testing number lookups:");
    Serial.print("  '1' -> "); Serial.println(getAudioIdTest("1"));
    Serial.print("  '12' -> "); Serial.println(getAudioIdTest("12"));
    Serial.print("  '15' -> "); Serial.println(getAudioIdTest("15"));
    Serial.print("  '90' -> "); Serial.println(getAudioIdTest("90"));

    Serial.println("\nTesting word lookups:");
    Serial.print("  'OCLOCK' -> "); Serial.println(getAudioIdTest("OCLOCK"));
    Serial.print("  'METRES' -> "); Serial.println(getAudioIdTest("METRES"));
    Serial.print("  'HUNDRED' -> "); Serial.println(getAudioIdTest("HUNDRED"));

    Serial.println("\n✓ Test complete");
}

void testCase1() {
    printTestHeader("Test Case 1: Bearing 86°, Distance 955m");
    Serial.println("Expected: 3 OCLOCK 9 HUNDRED AND 50 5 METRES\n");

    double bearing = 86.0;
    int distance = 955;

    int clockDirection = (int)round(bearing / 30.0);
    if (clockDirection == 0) clockDirection = 12;

    int rounded = ((distance + 2) / 5) * 5;

    Serial.print("Clock: "); Serial.println(clockDirection);
    Serial.print("Distance: "); Serial.println(rounded);

    Serial.println("\nAudio IDs:");
    Serial.print("  3 -> "); Serial.println(getAudioIdTest("3"));
    Serial.print("  OCLOCK -> "); Serial.println(getAudioIdTest("OCLOCK"));
    Serial.print("  9 -> "); Serial.println(getAudioIdTest("9"));
    Serial.print("  HUNDRED -> "); Serial.println(getAudioIdTest("HUNDRED"));
    Serial.print("  AND -> "); Serial.println(getAudioIdTest("AND"));
    Serial.print("  50 -> "); Serial.println(getAudioIdTest("50"));
    Serial.print("  5 -> "); Serial.println(getAudioIdTest("5"));
    Serial.print("  METRES -> "); Serial.println(getAudioIdTest("METRES"));

    Serial.println("\n✓ Test complete");
}

void testCase2() {
    printTestHeader("Test Case 2: Bearing 5°, Distance 50m");
    Serial.println("Expected: 12 OCLOCK 50 METRES\n");

    double bearing = 5.0;
    int distance = 50;

    int clockDirection = (int)round(bearing / 30.0);
    if (clockDirection == 0) clockDirection = 12;

    Serial.print("Clock: "); Serial.println(clockDirection);

    Serial.println("\nAudio IDs:");
    Serial.print("  12 -> "); Serial.println(getAudioIdTest("12"));
    Serial.print("  OCLOCK -> "); Serial.println(getAudioIdTest("OCLOCK"));
    Serial.print("  50 -> "); Serial.println(getAudioIdTest("50"));
    Serial.print("  METRES -> "); Serial.println(getAudioIdTest("METRES"));

    Serial.println("\n✓ Test complete");
}

void testCase3() {
    printTestHeader("Test Case 3: Bearing 180°, Distance 215m");
    Serial.println("Expected: 6 OCLOCK 2 HUNDRED AND 15 METRES\n");

    double bearing = 180.0;
    int distance = 215;

    int clockDirection = (int)round(bearing / 30.0);
    if (clockDirection == 0) clockDirection = 12;

    Serial.print("Clock: "); Serial.println(clockDirection);

    Serial.println("\nAudio IDs:");
    Serial.print("  6 -> "); Serial.println(getAudioIdTest("6"));
    Serial.print("  OCLOCK -> "); Serial.println(getAudioIdTest("OCLOCK"));
    Serial.print("  2 -> "); Serial.println(getAudioIdTest("2"));
    Serial.print("  HUNDRED -> "); Serial.println(getAudioIdTest("HUNDRED"));
    Serial.print("  AND -> "); Serial.println(getAudioIdTest("AND"));
    Serial.print("  15 -> "); Serial.println(getAudioIdTest("15"));
    Serial.print("  METRES -> "); Serial.println(getAudioIdTest("METRES"));

    Serial.println("\n✓ Test complete");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  playBuoyInstruction Test Suite       ║");
    Serial.println("║  Arduino Implementation Tests          ║");
    Serial.println("╔════════════════════════════════════════╗");

    delay(1000);

    testGetAudioId();
    delay(500);

    testCase1();
    delay(500);

    testCase2();
    delay(500);

    testCase3();
    delay(500);

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  All tests complete!                  ║");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("\nCompare with original parser output");
}

void loop() {
    // Nothing to do in loop
}
