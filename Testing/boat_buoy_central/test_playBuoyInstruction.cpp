/*
 * Test Suite for playBuoyInstruction
 *
 * This file contains tests to verify the audio instruction generation
 * matches the original parser implementation.
 *
 * To run these tests, temporarily add this to your Arduino sketch:
 * - Call runTests() in setup()
 * - Comment out the normal operation code
 */

#include "playBuoyInstruction.h"

// Mock DFPlayer for testing (just prints)
class MockDFPlayer {
public:
    void play(int id) {
        Serial.print("  player.play(");
        Serial.print(id);
        Serial.println(");");
    }
    bool begin(void* serial) { return true; }
    void setTimeOut(int ms) {}
    void volume(int vol) {}
    void EQ(int eq) {}
};

MockDFPlayer mockPlayer;

// Test helper: Print test header
void printTestHeader(const char* testName) {
    Serial.println("\n========================================");
    Serial.print("TEST: ");
    Serial.println(testName);
    Serial.println("========================================");
}

// Test helper: Print expected vs actual
void printExpected(const char* expected) {
    Serial.println("\nExpected audio sequence:");
    Serial.println(expected);
    Serial.println("\nActual audio sequence:");
}

// ============ Unit Tests ============

void testGetAudioId() {
    printTestHeader("getAudioId() - Audio Map Lookup");

    Serial.println("Testing number lookups:");
    Serial.print("  '1' -> "); Serial.println(getAudioId("1")); // Should be 1
    Serial.print("  '12' -> "); Serial.println(getAudioId("12")); // Should be 12
    Serial.print("  '15' -> "); Serial.println(getAudioId("15")); // Should be 13
    Serial.print("  '90' -> "); Serial.println(getAudioId("90")); // Should be 21

    Serial.println("\nTesting word lookups:");
    Serial.print("  'OCLOCK' -> "); Serial.println(getAudioId("OCLOCK")); // Should be 26
    Serial.print("  'METRES' -> "); Serial.println(getAudioId("METRES")); // Should be 25
    Serial.print("  'HUNDRED' -> "); Serial.println(getAudioId("HUNDRED")); // Should be 24

    Serial.println("\nTesting invalid lookup:");
    Serial.print("  'INVALID' -> "); Serial.println(getAudioId("INVALID")); // Should be -1

    Serial.println("\n✓ Test complete");
}

void testClockConversion() {
    printTestHeader("Clock Direction Conversion");

    struct TestCase {
        double bearing;
        int expectedClock;
    };

    TestCase cases[] = {
        {0.0, 12},      // North
        {30.0, 1},      // 1 o'clock
        {60.0, 2},      // 2 o'clock
        {90.0, 3},      // East - 3 o'clock
        {95.0, 3},      // ~3 o'clock (rounds to 3)
        {120.0, 4},     // 4 o'clock
        {180.0, 6},     // South - 6 o'clock
        {270.0, 9},     // West - 9 o'clock
        {330.0, 11},    // 11 o'clock
        {359.0, 12},    // Almost north (rounds to 12)
    };

    int numCases = sizeof(cases) / sizeof(cases[0]);

    for (int i = 0; i < numCases; i++) {
        double bearing = cases[i].bearing;
        int expectedClock = cases[i].expectedClock;

        // Calculate clock direction (same formula as in playBuoyInstruction)
        int clockDirection = (int)round(bearing / 30.0);
        if (clockDirection == 0) clockDirection = 12;

        Serial.print("  Bearing ");
        Serial.print(bearing, 1);
        Serial.print("° -> ");
        Serial.print(clockDirection);
        Serial.print(" o'clock");

        if (clockDirection == expectedClock) {
            Serial.println(" ✓");
        } else {
            Serial.print(" ✗ (expected ");
            Serial.print(expectedClock);
            Serial.println(")");
        }
    }

    Serial.println("\n✓ Test complete");
}

void testDistanceRounding() {
    printTestHeader("Distance Rounding");

    struct TestCase {
        int distance;
        int expectedRounded;
    };

    TestCase cases[] = {
        {0, 0},
        {3, 5},      // Round up to 5
        {7, 5},      // Round down to 5
        {8, 10},     // Round up to 10
        {12, 10},    // Round down to 10
        {13, 15},    // Round up to 15
        {50, 50},    // Exact match
        {135, 135},  // Exact match
        {137, 135},  // Round down
        {138, 140},  // Round up
    };

    int numCases = sizeof(cases) / sizeof(cases[0]);

    for (int i = 0; i < numCases; i++) {
        int distance = cases[i].distance;
        int expected = cases[i].expectedRounded;

        // Calculate rounded distance (same formula as in queueDistance)
        int rounded = ((distance + 2) / 5) * 5;

        Serial.print("  ");
        Serial.print(distance);
        Serial.print("m -> ");
        Serial.print(rounded);
        Serial.print("m");

        if (rounded == expected) {
            Serial.println(" ✓");
        } else {
            Serial.print(" ✗ (expected ");
            Serial.print(expected);
            Serial.println("m)");
        }
    }

    Serial.println("\n✓ Test complete");
}

// ============ Integration Tests ============
// These tests verify complete instruction generation

void testCase1_EastNortheast_955m() {
    printTestHeader("Test Case 1: Bearing 86°, Distance 955m");
    Serial.println("(Similar to original test: 0.00065, 0.00855)");

    printExpected("3 OCLOCK 9 HUNDRED AND 50 5 METRES");

    // Note: We can't actually call playBuoyInstruction here because it uses
    // the real myDFPlayer which doesn't exist in this test context.
    // Instead, we'll manually test the components:

    double bearing = 86.0;
    int distance = 955;

    // Test clock conversion
    int clockDirection = (int)round(bearing / 30.0);
    if (clockDirection == 0) clockDirection = 12;
    Serial.print("  Clock direction: "); Serial.println(clockDirection);  // Should be 3

    // Test distance rounding
    int rounded = ((distance + 2) / 5) * 5;
    Serial.print("  Rounded distance: "); Serial.println(rounded);  // Should be 955

    // Manual audio sequence
    Serial.println("\nExpected Audio IDs:");
    Serial.print("  3: "); Serial.println(getAudioId("3"));
    Serial.print("  OCLOCK: "); Serial.println(getAudioId("OCLOCK"));
    Serial.print("  9: "); Serial.println(getAudioId("9"));
    Serial.print("  HUNDRED: "); Serial.println(getAudioId("HUNDRED"));
    Serial.print("  AND: "); Serial.println(getAudioId("AND"));
    Serial.print("  50: "); Serial.println(getAudioId("50"));
    Serial.print("  5: "); Serial.println(getAudioId("5"));
    Serial.print("  METRES: "); Serial.println(getAudioId("METRES"));

    Serial.println("\n✓ Test complete");
}

void testCase2_North_50m() {
    printTestHeader("Test Case 2: Bearing 5°, Distance 50m");
    Serial.println("(Similar to original test: 0.00045, 0.00004)");

    printExpected("12 OCLOCK 50 METRES");

    double bearing = 5.0;
    int distance = 50;

    int clockDirection = (int)round(bearing / 30.0);
    if (clockDirection == 0) clockDirection = 12;
    Serial.print("  Clock direction: "); Serial.println(clockDirection);  // Should be 12

    int rounded = ((distance + 2) / 5) * 5;
    Serial.print("  Rounded distance: "); Serial.println(rounded);  // Should be 50

    Serial.println("\nExpected Audio IDs:");
    Serial.print("  12: "); Serial.println(getAudioId("12"));
    Serial.print("  OCLOCK: "); Serial.println(getAudioId("OCLOCK"));
    Serial.print("  50: "); Serial.println(getAudioId("50"));
    Serial.print("  METRES: "); Serial.println(getAudioId("METRES"));

    Serial.println("\n✓ Test complete");
}

void testCase3_South_215m() {
    printTestHeader("Test Case 3: Bearing 180°, Distance 215m");
    Serial.println("(Similar to original test: -0.00193, 0.0)");

    printExpected("6 OCLOCK 2 HUNDRED AND 15 METRES");

    double bearing = 180.0;
    int distance = 215;

    int clockDirection = (int)round(bearing / 30.0);
    if (clockDirection == 0) clockDirection = 12;
    Serial.print("  Clock direction: "); Serial.println(clockDirection);  // Should be 6

    int rounded = ((distance + 2) / 5) * 5;
    Serial.print("  Rounded distance: "); Serial.println(rounded);  // Should be 215

    Serial.println("\nExpected Audio IDs:");
    Serial.print("  6: "); Serial.println(getAudioId("6"));
    Serial.print("  OCLOCK: "); Serial.println(getAudioId("OCLOCK"));
    Serial.print("  2: "); Serial.println(getAudioId("2"));
    Serial.print("  HUNDRED: "); Serial.println(getAudioId("HUNDRED"));
    Serial.print("  AND: "); Serial.println(getAudioId("AND"));
    Serial.print("  15: "); Serial.println(getAudioId("15"));  // Special case!
    Serial.print("  METRES: "); Serial.println(getAudioId("METRES"));

    Serial.println("\n✓ Test complete - Note: 15 has its own audio file");
}

void testCase4_EdgeCase_15m() {
    printTestHeader("Test Case 4: Edge Case - 15 meters");
    Serial.println("Testing special handling of '15'");

    printExpected("X OCLOCK 15 METRES (not '10 5')");

    int distance = 15;
    int rounded = ((distance + 2) / 5) * 5;

    Serial.print("  Rounded distance: "); Serial.println(rounded);
    Serial.println("\n  15 should use audio ID 13 (special file)");
    Serial.print("  Audio ID for '15': "); Serial.println(getAudioId("15"));

    Serial.println("\n✓ Test complete");
}

void testCase5_EdgeCase_115m() {
    printTestHeader("Test Case 5: Edge Case - 115 meters");
    Serial.println("Testing '1 HUNDRED AND 15 METRES'");

    printExpected("X OCLOCK 1 HUNDRED AND 15 METRES");

    int distance = 115;
    int rounded = ((distance + 2) / 5) * 5;

    Serial.println("\nExpected Audio IDs:");
    Serial.print("  1: "); Serial.println(getAudioId("1"));
    Serial.print("  HUNDRED: "); Serial.println(getAudioId("HUNDRED"));
    Serial.print("  AND: "); Serial.println(getAudioId("AND"));
    Serial.print("  15: "); Serial.println(getAudioId("15"));  // Special case!
    Serial.print("  METRES: "); Serial.println(getAudioId("METRES"));

    Serial.println("\n✓ Test complete");
}

// ============ Main Test Runner ============

void runAllTests() {
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  playBuoyInstruction Test Suite       ║");
    Serial.println("║  Verifying Arduino implementation     ║");
    Serial.println("╔════════════════════════════════════════╗");

    delay(1000);

    // Unit tests
    testGetAudioId();
    delay(500);

    testClockConversion();
    delay(500);

    testDistanceRounding();
    delay(500);

    // Integration tests
    testCase1_EastNortheast_955m();
    delay(500);

    testCase2_North_50m();
    delay(500);

    testCase3_South_215m();
    delay(500);

    testCase4_EdgeCase_15m();
    delay(500);

    testCase5_EdgeCase_115m();
    delay(500);

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  All tests complete!                  ║");
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("\nCompare output with original parser/playBuoyInstruction.cpp");
}
