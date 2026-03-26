import Foundation
import HealthKit
import Combine

class HealthKitManager: ObservableObject {
    @Published var heartRate: Double = 0
    @Published var isAuthorized = false

    private let healthStore = HKHealthStore()
    private var heartRateQuery: HKAnchoredObjectQuery?

    private let heartRateType = HKQuantityType.quantityType(forIdentifier: .heartRate)!
    private let activeEnergyType = HKQuantityType.quantityType(forIdentifier: .activeEnergyBurned)!
    private let workoutType = HKObjectType.workoutType()

    // These may not be available on all devices — save attempts are best-effort
    private let cyclingPowerType = HKQuantityType.quantityType(forIdentifier: .cyclingPower)
    private let cyclingCadenceType = HKQuantityType.quantityType(forIdentifier: .cyclingCadence)

    func requestAuthorization() {
        guard HKHealthStore.isHealthDataAvailable() else { return }

        let readTypes: Set<HKObjectType> = [heartRateType, activeEnergyType, workoutType]
        var writeTypes: Set<HKSampleType> = [workoutType, activeEnergyType, heartRateType]
        if let p = cyclingPowerType { writeTypes.insert(p) }
        if let c = cyclingCadenceType { writeTypes.insert(c) }

        healthStore.requestAuthorization(toShare: writeTypes, read: readTypes) { success, _ in
            DispatchQueue.main.async {
                self.isAuthorized = success
                if success { self.startHeartRateStream() }
            }
        }
    }

    func startHeartRateStream() {
        stopHeartRateStream()
        let predicate = HKQuery.predicateForSamples(withStart: Date(), end: nil, options: .strictStartDate)
        let query = HKAnchoredObjectQuery(
            type: heartRateType, predicate: predicate, anchor: nil, limit: HKObjectQueryNoLimit
        ) { [weak self] _, samples, _, _, _ in
            self?.processHeartRateSamples(samples)
        }
        query.updateHandler = { [weak self] _, samples, _, _, _ in
            self?.processHeartRateSamples(samples)
        }
        heartRateQuery = query
        healthStore.execute(query)
    }

    func stopHeartRateStream() {
        if let query = heartRateQuery {
            healthStore.stop(query)
            heartRateQuery = nil
        }
    }

    private func processHeartRateSamples(_ samples: [HKSample]?) {
        guard let samples = samples as? [HKQuantitySample], let latest = samples.last else { return }
        let bpm = latest.quantity.doubleValue(for: HKUnit.count().unitDivided(by: .minute()))
        DispatchQueue.main.async { self.heartRate = bpm }
    }

    // MARK: - Save Workout

    func saveWorkout(
        start: Date,
        end: Date,
        powerSamples: [(Date, Double)],
        cadenceSamples: [(Date, Double)],
        heartRateSamples: [(Date, Double)],
        totalCalories: Double,
        completion: @escaping (Bool, Error?) -> Void
    ) {
        let safeEnd = end > start ? end : start.addingTimeInterval(1)

        let config = HKWorkoutConfiguration()
        config.activityType = .cycling
        config.locationType = .indoor

        let builder = HKWorkoutBuilder(healthStore: healthStore, configuration: config, device: nil)

        // Thread-safe timeout
        var completed = false
        let lock = NSLock()
        let safeComplete: (Bool, Error?) -> Void = { success, error in
            lock.lock()
            guard !completed else { lock.unlock(); return }
            completed = true
            lock.unlock()
            completion(success, error)
        }

        DispatchQueue.global().asyncAfter(deadline: .now() + 15) {
            safeComplete(false, NSError(domain: "PowerMeter", code: -1,
                userInfo: [NSLocalizedDescriptionKey: "HealthKit save timed out. Check Health permissions in Settings > Health > Data Access."]))
        }

        builder.beginCollection(withStart: start) { success, error in
            guard success else {
                safeComplete(false, error ?? NSError(domain: "PowerMeter", code: -2,
                    userInfo: [NSLocalizedDescriptionKey: "Could not begin workout. Grant Health permissions in Settings."]))
                return
            }

            var samples: [HKQuantitySample] = []

            // Heart rate
            let hrUnit = HKUnit.count().unitDivided(by: .minute())
            for (date, value) in heartRateSamples where value > 0 {
                let d = Swift.min(Swift.max(date, start), safeEnd)
                samples.append(HKQuantitySample(type: self.heartRateType,
                    quantity: HKQuantity(unit: hrUnit, doubleValue: value), start: d, end: d))
            }

            // Cycling power
            if let powerType = self.cyclingPowerType {
                let wattUnit = HKUnit.watt()
                for (date, value) in powerSamples where value > 0 {
                    let d = Swift.min(Swift.max(date, start), safeEnd)
                    samples.append(HKQuantitySample(type: powerType,
                        quantity: HKQuantity(unit: wattUnit, doubleValue: value), start: d, end: d))
                }
            }

            // Cycling cadence
            if let cadenceType = self.cyclingCadenceType {
                let rpmUnit = HKUnit.count().unitDivided(by: .minute())
                for (date, value) in cadenceSamples where value > 0 {
                    let d = Swift.min(Swift.max(date, start), safeEnd)
                    samples.append(HKQuantitySample(type: cadenceType,
                        quantity: HKQuantity(unit: rpmUnit, doubleValue: value), start: d, end: d))
                }
            }

            // Active energy
            if totalCalories > 0 {
                samples.append(HKQuantitySample(type: self.activeEnergyType,
                    quantity: HKQuantity(unit: .kilocalorie(), doubleValue: totalCalories),
                    start: start, end: safeEnd))
            }

            let finish = {
                builder.endCollection(withEnd: safeEnd) { _, endErr in
                    builder.finishWorkout { workout, finErr in
                        safeComplete(workout != nil, finErr ?? endErr)
                    }
                }
            }

            if samples.isEmpty {
                finish()
            } else {
                builder.add(samples) { addOk, _ in
                    if !addOk {
                        // If adding all samples fails, try with just energy
                        let energyOnly = samples.filter { $0.quantityType == self.activeEnergyType }
                        if energyOnly.isEmpty {
                            finish()
                        } else {
                            builder.add(energyOnly) { _, _ in finish() }
                        }
                    } else {
                        finish()
                    }
                }
            }
        }
    }
}
