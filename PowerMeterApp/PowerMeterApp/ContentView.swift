import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    var body: some View {
        GeometryReader { geo in
            // Inside FullScreenContainer (safeAreaRegions=[]),
            // geo.size should be the true full size
            let w = geo.size.width
            let h = geo.size.height

            if showingSummary, let summary = workoutSession.lastSummary {
                WorkoutSummaryView(summary: summary) {
                    showingSummary = false
                }
                .frame(width: w, height: h)
                .background(Color.black)
            } else {
                BikeComputerView(
                    screenW: w,
                    screenH: h,
                    onWorkoutEnd: {
                        if workoutSession.lastSummary != nil {
                            showingSummary = true
                        }
                    }
                )
            }
        }
        .background(Color.black)
    }
}
