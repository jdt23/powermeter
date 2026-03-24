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
    private let cyclingPowerType = HKQuantityType.quantityType(forIdentifier: .cyclingPower)!
    private let cyclingCadenceType = HKQuantityType.quantityType(forIdentifier: .cyclingCadence)!
    private let workoutType = HKObjectType.workoutType()

    func requestAuthorization() {
        guard HKHealthStore.isHealthDataAvailable() else { return }

        let readTypes: Set<HKObjectType> = [heartRateType, activeEnergyType, workoutType]
        let writeTypes: Set<HKSampleType> = [
            workoutType, activeEnergyType, cyclingPowerType, cyclingCadenceType, heartRateType
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

            var allSamples: [HKQuantitySample] = []

            // Power samples
            let powerUnit = HKUnit.watt()
            for (date, value) in powerSamples where value > 0 {
                let quantity = HKQuantity(unit: powerUnit, doubleValue: value)
                let sample = HKQuantitySample(
                    type: self.cyclingPowerType,
                    quantity: quantity,
                    start: date,
                    end: date
                )
                allSamples.append(sample)
            }

            // Cadence samples (HealthKit uses rev/min for cycling cadence)
            let cadenceUnit = HKUnit.count().unitDivided(by: .minute())
            for (date, value) in cadenceSamples where value > 0 {
                let quantity = HKQuantity(unit: cadenceUnit, doubleValue: value)
                let sample = HKQuantitySample(
                    type: self.cyclingCadenceType,
                    quantity: quantity,
                    start: date,
                    end: date
                )
                allSamples.append(sample)
            }

            // Heart rate samples
            let hrUnit = HKUnit.count().unitDivided(by: .minute())
            for (date, value) in heartRateSamples where value > 0 {
                let quantity = HKQuantity(unit: hrUnit, doubleValue: value)
                let sample = HKQuantitySample(
                    type: self.heartRateType,
                    quantity: quantity,
                    start: date,
                    end: date
                )
                allSamples.append(sample)
            }

            // Active energy
            if totalCalories > 0 {
                let calQuantity = HKQuantity(unit: .kilocalorie(), doubleValue: totalCalories)
                let calSample = HKQuantitySample(
                    type: self.activeEnergyType,
                    quantity: calQuantity,
                    start: start,
                    end: end
                )
                allSamples.append(calSample)
            }

            builder.add(allSamples) { _, _ in
                builder.endCollection(withEnd: end) { _, _ in
                    builder.finishWorkout { workout, error in
                        completion(workout != nil, error)
                    }
                }
            }
        }
    }
}
