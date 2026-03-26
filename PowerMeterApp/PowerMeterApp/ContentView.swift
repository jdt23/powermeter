import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    private var W: CGFloat { UIScreen.main.bounds.width }
    private var H: CGFloat { UIScreen.main.bounds.height }

    var body: some View {
        if showingSummary, let summary = workoutSession.lastSummary {
            WorkoutSummaryView(summary: summary) {
                showingSummary = false
            }
            .frame(width: W, height: H)
            .background(Color.black)
        } else {
            BikeComputerView(
                screenW: W,
                screenH: H,
                onWorkoutEnd: {
                    if workoutSession.lastSummary != nil {
                        showingSummary = true
                    }
                }
            )
        }
    }
}
