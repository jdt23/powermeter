import SwiftUI

@main
struct PowerMeterWatchApp: App {
    @StateObject private var workoutManager = WatchWorkoutManager()
    @StateObject private var connectivityManager = WatchConnectivityManager()

    var body: some Scene {
        WindowGroup {
            WorkoutView()
                .environmentObject(workoutManager)
                .environmentObject(connectivityManager)
                .onAppear {
                    connectivityManager.workoutManager = workoutManager
                    workoutManager.connectivityManager = connectivityManager
                }
        }
    }
}
