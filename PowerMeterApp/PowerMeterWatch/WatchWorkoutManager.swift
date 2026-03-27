import Foundation
import HealthKit
import Combine

class WatchWorkoutManager: NSObject, ObservableObject {
    @Published var isActive = false
    @Published var isPaused = false
    @Published var heartRate: Double = 0
    @Published var activeCalories: Double = 0
    @Published var power: Int = 0
    @Published var cadence: Int = 0
    @Published var resistance: Int = 0
    @Published var elapsed: TimeInterval = 0
    @Published var speed: Double = 0
    @Published var distance: Double = 0
    @Published var calories: Double = 0

    var connectivityManager: WatchConnectivityManager?

    private let healthStore = HKHealthStore()
    private var session: HKWorkoutSession?
    private var builder: HKLiveWorkoutBuilder?
    private var timer: AnyCancellable?
    private var startDate: Date?

    private let cyclingPowerType = HKQuantityType.quantityType(forIdentifier: .cyclingPower)!
    private let cyclingCadenceType = HKQuantityType.quantityType(forIdentifier: .cyclingCadence)!

    func requestAuthorization() {
        let readTypes: Set<HKObjectType> = [
            HKQuantityType.quantityType(forIdentifier: .heartRate)!,
            HKQuantityType.quantityType(forIdentifier: .activeEnergyBurned)!,
            HKObjectType.workoutType()
        ]
        let writeTypes: Set<HKSampleType> = [
            HKQuantityType.quantityType(forIdentifier: .heartRate)!,
            HKQuantityType.quantityType(forIdentifier: .activeEnergyBurned)!,
            cyclingPowerType,
            cyclingCadenceType,
            HKObjectType.workoutType()
        ]

        healthStore.requestAuthorization(toShare: writeTypes, read: readTypes) { _, _ in }
    }

    func startWorkout() {
        let config = HKWorkoutConfiguration()
        config.activityType = .cycling
        config.locationType = .indoor

        do {
            session = try HKWorkoutSession(healthStore: healthStore, configuration: config)
            builder = session?.associatedWorkoutBuilder()
        } catch {
            return
        }

        guard let session = session, let builder = builder else { return }

        session.delegate = self
        builder.delegate = self
        builder.dataSource = HKLiveWorkoutDataSource(healthStore: healthStore, workoutConfiguration: config)

        let startDate = Date()
        self.startDate = startDate

        session.startActivity(with: startDate)
        builder.beginCollection(withStart: startDate) { _, _ in }

        DispatchQueue.main.async {
            self.isActive = true
            self.isPaused = false
            self.startTimer()
        }
    }

    func pauseWorkout() {
        session?.pause()
        DispatchQueue.main.async {
            self.isPaused = true
            self.timer?.cancel()
        }
    }

    func resumeWorkout() {
        session?.resume()
        DispatchQueue.main.async {
            self.isPaused = false
            self.startTimer()
        }
    }

    func stopWorkout() {
        timer?.cancel()
        session?.end()

        builder?.endCollection(withEnd: Date()) { [weak self] _, _ in
            self?.builder?.finishWorkout { workout, _ in
                DispatchQueue.main.async {
                    self?.isActive = false
                    self?.isPaused = false
                    self?.heartRate = 0
                    self?.activeCalories = 0
                    self?.power = 0
                    self?.cadence = 0
                    self?.resistance = 0
                    self?.elapsed = 0
                    // Notify iPhone that Watch saved the workout
                    if workout != nil {
                        self?.connectivityManager?.sendWorkoutSaved()
                    }
                }
            }
        }
    }

    func addPowerSample(_ watts: Double) {
        guard let builder = builder, watts > 0 else { return }
        let quantity = HKQuantity(unit: .watt(), doubleValue: watts)
        let sample = HKQuantitySample(type: cyclingPowerType, quantity: quantity, start: Date(), end: Date())
        builder.add([sample]) { _, _ in }
    }

    func addCadenceSample(_ rpm: Double) {
        guard let builder = builder, rpm > 0 else { return }
        let unit = HKUnit.count().unitDivided(by: .minute())
        let quantity = HKQuantity(unit: unit, doubleValue: rpm)
        let sample = HKQuantitySample(type: cyclingCadenceType, quantity: quantity, start: Date(), end: Date())
        builder.add([sample]) { _, _ in }
    }

    func updateMetrics(power: Int, cadence: Int, resistance: Int,
                        speed: Double = 0, distance: Double = 0, calories: Double = 0) {
        DispatchQueue.main.async {
            self.power = power
            self.cadence = cadence
            self.resistance = resistance
            self.speed = speed
            self.distance = distance
            self.calories = calories
        }
        addPowerSample(Double(power))
        addCadenceSample(Double(cadence))
    }

    private func startTimer() {
        timer = Timer.publish(every: 1, on: .main, in: .common)
            .autoconnect()
            .sink { [weak self] _ in
                guard let start = self?.startDate else { return }
                self?.elapsed = Date().timeIntervalSince(start)
            }
    }
}

// MARK: - HKWorkoutSessionDelegate
extension WatchWorkoutManager: HKWorkoutSessionDelegate {
    func workoutSession(_ workoutSession: HKWorkoutSession, didChangeTo toState: HKWorkoutSessionState,
                         from fromState: HKWorkoutSessionState, date: Date) {
    }

    func workoutSession(_ workoutSession: HKWorkoutSession, didFailWithError error: Error) {
    }
}

// MARK: - HKLiveWorkoutBuilderDelegate
extension WatchWorkoutManager: HKLiveWorkoutBuilderDelegate {
    func workoutBuilderDidCollectEvent(_ workoutBuilder: HKLiveWorkoutBuilder) {
    }

    func workoutBuilder(_ workoutBuilder: HKLiveWorkoutBuilder, didCollectDataOf collectedTypes: Set<HKSampleType>) {
        for type in collectedTypes {
            guard let quantityType = type as? HKQuantityType else { continue }

            if let statistics = workoutBuilder.statistics(for: quantityType) {
                DispatchQueue.main.async {
                    switch quantityType {
                    case HKQuantityType.quantityType(forIdentifier: .heartRate):
                        let hrUnit = HKUnit.count().unitDivided(by: .minute())
                        if let value = statistics.mostRecentQuantity()?.doubleValue(for: hrUnit) {
                            self.heartRate = value
                            self.connectivityManager?.sendHeartRate(value)
                        }
                    case HKQuantityType.quantityType(forIdentifier: .activeEnergyBurned):
                        let calUnit = HKUnit.kilocalorie()
                        if let value = statistics.sumQuantity()?.doubleValue(for: calUnit) {
                            self.activeCalories = value
                        }
                    default:
                        break
                    }
                }
            }
        }
    }
}
