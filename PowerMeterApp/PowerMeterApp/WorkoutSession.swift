import Foundation
import Combine

enum WorkoutState {
    case idle
    case active
    case paused
}

struct WorkoutSummary {
    let startDate: Date
    let endDate: Date
    let duration: TimeInterval
    let averagePower: Double
    let maxPower: Double
    let averageCadence: Double
    let averageHeartRate: Double
    let maxHeartRate: Double
    let totalCalories: Double
    let powerSampleCount: Int
    let cadenceSampleCount: Int
    let heartRateSampleCount: Int
}

class WorkoutSession: ObservableObject {
    @Published var state: WorkoutState = .idle
    @Published var elapsed: TimeInterval = 0
    @Published var averagePower: Double = 0
    @Published var maxPower: Double = 0
    @Published var averageCadence: Double = 0
    @Published var averageHeartRate: Double = 0
    @Published var maxHeartRate: Double = 0
    @Published var totalCalories: Double = 0
    @Published var lastSummary: WorkoutSummary?

    private var startDate: Date?
    private var pauseDate: Date?
    private var accumulatedPause: TimeInterval = 0
    private var timer: AnyCancellable?

    // Sample storage for HealthKit export
    private(set) var powerSamples: [(Date, Double)] = []
    private(set) var cadenceSamples: [(Date, Double)] = []
    private(set) var heartRateSamples: [(Date, Double)] = []

    private var powerSum: Double = 0
    private var powerCount: Int = 0
    private var cadenceSum: Double = 0
    private var cadenceCount: Int = 0
    private var hrSum: Double = 0
    private var hrCount: Int = 0

    var workoutStartDate: Date? { startDate }

    func start() {
        reset()
        startDate = Date()
        state = .active
        startTimer()
    }

    func pause() {
        guard state == .active else { return }
        pauseDate = Date()
        state = .paused
        timer?.cancel()
    }

    func resume() {
        guard state == .paused, let pauseStart = pauseDate else { return }
        accumulatedPause += Date().timeIntervalSince(pauseStart)
        pauseDate = nil
        state = .active
        startTimer()
    }

    func stop() -> WorkoutSummary? {
        timer?.cancel()
        state = .idle

        guard let start = startDate else { return nil }
        let end = Date()

        let summary = WorkoutSummary(
            startDate: start,
            endDate: end,
            duration: elapsed,
            averagePower: averagePower,
            maxPower: maxPower,
            averageCadence: averageCadence,
            averageHeartRate: averageHeartRate,
            maxHeartRate: maxHeartRate,
            totalCalories: totalCalories,
            powerSampleCount: powerSamples.filter { $0.1 > 0 }.count,
            cadenceSampleCount: cadenceSamples.filter { $0.1 > 0 }.count,
            heartRateSampleCount: heartRateSamples.filter { $0.1 > 0 }.count
        )
        lastSummary = summary
        return summary
    }

    func recordPower(_ value: UInt16) {
        guard state == .active else { return }
        let v = Double(value)
        powerSamples.append((Date(), v))
        powerSum += v
        powerCount += 1
        averagePower = powerSum / Double(powerCount)
        if v > maxPower { maxPower = v }

        // Estimate calories: Power(W) * time(s) / 4184 * 4 (assuming ~25% efficiency)
        // Per 2-second sample: watts * 2 / 4184 * 4
        totalCalories = powerSum * 2.0 / 4184.0 * 4.0
    }

    func recordCadence(_ value: UInt8) {
        guard state == .active else { return }
        let v = Double(value)
        cadenceSamples.append((Date(), v))
        cadenceSum += v
        cadenceCount += 1
        averageCadence = cadenceSum / Double(cadenceCount)
    }

    func recordHeartRate(_ value: Double) {
        guard state == .active, value > 0 else { return }
        heartRateSamples.append((Date(), value))
        hrSum += value
        hrCount += 1
        averageHeartRate = hrSum / Double(hrCount)
        if value > maxHeartRate { maxHeartRate = value }
    }

    private func startTimer() {
        timer = Timer.publish(every: 1, on: .main, in: .common)
            .autoconnect()
            .sink { [weak self] _ in
                self?.updateElapsed()
            }
    }

    private func updateElapsed() {
        guard let start = startDate else { return }
        elapsed = Date().timeIntervalSince(start) - accumulatedPause
    }

    private func reset() {
        elapsed = 0
        averagePower = 0
        maxPower = 0
        averageCadence = 0
        averageHeartRate = 0
        maxHeartRate = 0
        totalCalories = 0
        powerSamples = []
        cadenceSamples = []
        heartRateSamples = []
        powerSum = 0
        powerCount = 0
        cadenceSum = 0
        cadenceCount = 0
        hrSum = 0
        hrCount = 0
        accumulatedPause = 0
        pauseDate = nil
        lastSummary = nil
    }
}
