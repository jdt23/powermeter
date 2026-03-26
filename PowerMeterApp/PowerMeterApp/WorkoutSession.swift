import Foundation
import Combine

enum WorkoutState {
    case idle
    case active
    case paused
}

enum PowerZone: Int, CaseIterable {
    case z1 = 1, z2, z3, z4, z5, z6

    var label: String {
        switch self {
        case .z1: return "RECOVERY"
        case .z2: return "ENDURANCE"
        case .z3: return "TEMPO"
        case .z4: return "THRESHOLD"
        case .z5: return "VO2 MAX"
        case .z6: return "ANAEROBIC"
        }
    }

    var color: String {
        switch self {
        case .z1: return "gray"
        case .z2: return "blue"
        case .z3: return "green"
        case .z4: return "yellow"
        case .z5: return "orange"
        case .z6: return "red"
        }
    }

    static func zone(watts: Double, ftp: Double) -> PowerZone {
        let pct = watts / ftp
        if pct < 0.55 { return .z1 }
        if pct < 0.75 { return .z2 }
        if pct < 0.90 { return .z3 }
        if pct < 1.05 { return .z4 }
        if pct < 1.20 { return .z5 }
        return .z6
    }
}

struct WorkoutSummary {
    let startDate: Date
    let endDate: Date
    let duration: TimeInterval
    let averagePower: Double
    let maxPower: Double
    let normalizedPower: Double
    let intensityFactor: Double
    let tss: Double
    let averageCadence: Double
    let maxCadence: Double
    let averageResistance: Double
    let maxResistance: Double
    let averageHeartRate: Double
    let maxHeartRate: Double
    let averageSpeed: Double
    let maxSpeed: Double
    let totalDistance: Double
    let totalCalories: Double
    let powerSampleCount: Int
    let cadenceSampleCount: Int
    let heartRateSampleCount: Int
    let ftp: Double
}

class WorkoutSession: ObservableObject {
    @Published var state: WorkoutState = .idle
    @Published var elapsed: TimeInterval = 0
    @Published var averagePower: Double = 0
    @Published var maxPower: Double = 0
    @Published var averageCadence: Double = 0
    @Published var maxCadence: Double = 0
    @Published var averageResistance: Double = 0
    @Published var maxResistance: Double = 0
    @Published var averageHeartRate: Double = 0
    @Published var maxHeartRate: Double = 0
    @Published var totalCalories: Double = 0
    @Published var lastSummary: WorkoutSummary?

    // New metrics
    @Published var speed: Double = 0          // mph
    @Published var averageSpeed: Double = 0
    @Published var maxSpeed: Double = 0
    @Published var distance: Double = 0       // miles
    @Published var normalizedPower: Double = 0
    @Published var intensityFactor: Double = 0
    @Published var tss: Double = 0
    @Published var powerZone: PowerZone = .z1

    // FTP — default 200W, user can adjust
    var ftp: Double = 200.0

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
    private var resistanceSum: Double = 0
    private var resistanceCount: Int = 0
    private var hrSum: Double = 0
    private var hrCount: Int = 0
    private var speedSum: Double = 0
    private var speedCount: Int = 0
    private var lastRecordTime: Date?

    // For Normalized Power: 30-second rolling average
    private var powerWindow: [Double] = []  // raw power values (one per 2s sample)
    private var np4Sum: Double = 0          // sum of (30s_avg)^4
    private var np4Count: Int = 0

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
            normalizedPower: normalizedPower,
            intensityFactor: intensityFactor,
            tss: tss,
            averageCadence: averageCadence,
            maxCadence: maxCadence,
            averageResistance: averageResistance,
            maxResistance: maxResistance,
            averageHeartRate: averageHeartRate,
            maxHeartRate: maxHeartRate,
            averageSpeed: averageSpeed,
            maxSpeed: maxSpeed,
            totalDistance: distance,
            totalCalories: totalCalories,
            powerSampleCount: powerSamples.filter { $0.1 > 0 }.count,
            cadenceSampleCount: cadenceSamples.filter { $0.1 > 0 }.count,
            heartRateSampleCount: heartRateSamples.filter { $0.1 > 0 }.count,
            ftp: ftp
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

        // Calories: watts * seconds / 4184 * 4 (25% metabolic efficiency)
        totalCalories = powerSum * 2.0 / 4184.0 * 4.0

        // Power zone
        powerZone = PowerZone.zone(watts: v, ftp: ftp)

        // Normalized Power: 30-second rolling average
        powerWindow.append(v)
        let windowSize = 15  // 15 samples * 2s = 30 seconds
        if powerWindow.count > windowSize {
            powerWindow.removeFirst(powerWindow.count - windowSize)
        }
        if powerWindow.count >= windowSize {
            let rollingAvg = powerWindow.reduce(0, +) / Double(windowSize)
            np4Sum += pow(rollingAvg, 4)
            np4Count += 1
            normalizedPower = pow(np4Sum / Double(np4Count), 0.25)

            // IF = NP / FTP
            intensityFactor = ftp > 0 ? normalizedPower / ftp : 0

            // TSS = (sec * NP * IF) / (FTP * 3600) * 100
            if ftp > 0 && elapsed > 0 {
                tss = (elapsed * normalizedPower * intensityFactor) / (ftp * 3600.0) * 100.0
            }
        }
    }

    func recordCadence(_ value: UInt8) {
        guard state == .active else { return }
        let v = Double(value)
        cadenceSamples.append((Date(), v))
        cadenceSum += v
        cadenceCount += 1
        averageCadence = cadenceSum / Double(cadenceCount)
        if v > maxCadence { maxCadence = v }
    }

    func recordResistance(_ value: UInt8) {
        guard state == .active else { return }
        let v = Double(value)
        resistanceSum += v
        resistanceCount += 1
        averageResistance = resistanceSum / Double(resistanceCount)
        if v > maxResistance { maxResistance = v }
    }

    func recordHeartRate(_ value: Double) {
        guard state == .active, value > 0 else { return }
        heartRateSamples.append((Date(), value))
        hrSum += value
        hrCount += 1
        averageHeartRate = hrSum / Double(hrCount)
        if value > maxHeartRate { maxHeartRate = value }
    }

    /// Compute speed from cadence + resistance using Peloton formula, accumulate distance
    func recordSpeed(cadence: UInt8, resistance: UInt8) {
        guard state == .active else { return }
        let cad = Double(cadence)
        let res = Double(resistance)

        // Speed (mph) from Peloton formula: (Cadence - 35)^0.4 * (Resistance/100) * 9 + 0.4
        var spd: Double = 0
        if cad > 35 && res > 0 {
            spd = pow(cad - 35.0, 0.4) * (res / 100.0) * 9.0 + 0.4
        }
        speed = spd

        speedSum += spd
        speedCount += 1
        averageSpeed = speedSum / Double(speedCount)
        if spd > maxSpeed { maxSpeed = spd }

        // Accumulate distance: speed (mph) * time (hours)
        let now = Date()
        if let last = lastRecordTime {
            let dt = now.timeIntervalSince(last) / 3600.0  // hours
            distance += spd * dt
        }
        lastRecordTime = now
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
        maxCadence = 0
        averageResistance = 0
        maxResistance = 0
        averageHeartRate = 0
        maxHeartRate = 0
        totalCalories = 0
        speed = 0
        averageSpeed = 0
        maxSpeed = 0
        distance = 0
        normalizedPower = 0
        intensityFactor = 0
        tss = 0
        powerZone = .z1
        powerSamples = []
        cadenceSamples = []
        heartRateSamples = []
        powerSum = 0
        powerCount = 0
        cadenceSum = 0
        cadenceCount = 0
        resistanceSum = 0
        resistanceCount = 0
        hrSum = 0
        hrCount = 0
        speedSum = 0
        speedCount = 0
        powerWindow = []
        np4Sum = 0
        np4Count = 0
        accumulatedPause = 0
        pauseDate = nil
        lastSummary = nil
        lastRecordTime = nil
    }
}
