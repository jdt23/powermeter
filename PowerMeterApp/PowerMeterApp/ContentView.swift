import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    var body: some View {
        Color.black
            .ignoresSafeArea(.all)
            .overlay {
                if showingSummary, let summary = workoutSession.lastSummary {
                    WorkoutSummaryView(summary: summary) {
                        showingSummary = false
                    }
                } else {
                    BikeComputerView(onWorkoutEnd: {
                        if workoutSession.lastSummary != nil {
                            showingSummary = true
                        }
                    })
                }
            }
            .ignoresSafeArea(.all)
            .statusBarHidden(true)
            .persistentSystemOverlays(.hidden)
    }
}
