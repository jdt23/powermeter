import SwiftUI

@main
struct PowerMeterApp: App {
    @StateObject private var bleManager = BLEManager()
    @StateObject private var healthKitManager = HealthKitManager()
    @StateObject private var workoutSession = WorkoutSession()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(bleManager)
                .environmentObject(healthKitManager)
                .environmentObject(workoutSession)
                .onAppear {
                    healthKitManager.requestAuthorization()
                }
                .preferredColorScheme(.dark)
        }
    }
}
