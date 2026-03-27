import SwiftUI

@main
struct PowerMeterApp: App {
    @StateObject private var bleManager = BLEManager()
    @StateObject private var healthKitManager = HealthKitManager()
    @StateObject private var workoutSession = WorkoutSession()
    @StateObject private var connectivityManager = PhoneConnectivityManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(bleManager)
                .environmentObject(healthKitManager)
                .environmentObject(workoutSession)
                .environmentObject(connectivityManager)
                .onAppear {
                    healthKitManager.requestAuthorization()
                    connectivityManager.activateWatch()
                }
                .preferredColorScheme(.dark)
        }
    }
}
