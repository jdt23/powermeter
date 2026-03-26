import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    var body: some View {
        Group {
            if showingSummary, let summary = workoutSession.lastSummary {
                WorkoutSummaryView(summary: summary) {
                    showingSummary = false
                }
                .background(Color.black)
            } else {
                BikeComputerView(onWorkoutEnd: {
                    if workoutSession.lastSummary != nil {
                        showingSummary = true
                    }
                })
            }
        }
        .ignoresSafeArea()
        .statusBarHidden(true)
    }
}
