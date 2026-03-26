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

    func requestAuthorization() {
        guard HKHealthStore.isHealthDataAvailable() else { return }

        let readTypes: Set<HKObjectType> = [heartRateType, activeEnergyType, workoutType]
        let writeTypes: Set<HKSampleType> = [
            workoutType, activeEnergyType, heartRateType
        ]

        healthStore.requestAuthorization(toShare: writeTypes, read: readTypes) { success, _ in
            DispatchQueue.main.async {
                self.isAuthorized = success
                if success {
                    self.startHeartRateStream()
                }
            }
        }
    }

    func startHeartRateStream() {
        stopHeartRateStream()

        let predicate = HKQuery.predicateForSamples(
            withStart: Date(),
            end: nil,
            options: .strictStartDate
        )

        let query = HKAnchoredObjectQuery(
            type: heartRateType,
            predicate: predicate,
            anchor: nil,
            limit: HKObjectQueryNoLimit
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
        DispatchQueue.main.async {
            self.heartRate = bpm
        }
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
        let config = HKWorkoutConfiguration()
        config.activityType = .cycling
        config.locationType = .indoor

        let builder = HKWorkoutBuilder(healthStore: healthStore, configuration: config, device: nil)

        builder.beginCollection(withStart: start) { success, error in
            guard success else {
                completion(false, error)
                return
            }

            // Build samples - only use standard types that are reliably writable
            var allSamples: [HKQuantitySample] = []

            // Heart rate samples
            let hrUnit = HKUnit.count().unitDivided(by: .minute())
            for (date, value) in heartRateSamples where value > 0 {
                let sample = HKQuantitySample(
                    type: self.heartRateType,
                    quantity: HKQuantity(unit: hrUnit, doubleValue: value),
                    start: date, end: date
                )
                allSamples.append(sample)
            }

            // Active energy as a single summary sample
            if totalCalories > 0 {
                let sample = HKQuantitySample(
                    type: self.activeEnergyType,
                    quantity: HKQuantity(unit: .kilocalorie(), doubleValue: totalCalories),
                    start: start, end: end
                )
                allSamples.append(sample)
            }

            let finishBlock = {
                builder.endCollection(withEnd: end) { endOk, endErr in
                    guard endOk else {
                        completion(false, endErr)
                        return
                    }
                    builder.finishWorkout { workout, finishErr in
                        completion(workout != nil, finishErr)
                    }
                }
            }

            if allSamples.isEmpty {
                finishBlock()
            } else {
                builder.add(allSamples) { addOk, addErr in
                    if !addOk {
                        // Try finishing without samples rather than failing entirely
                        finishBlock()
                    } else {
                        finishBlock()
                    }
                }
            }
        }
    }
}
