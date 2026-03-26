import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    var body: some View {
        GeometryReader { geo in
            if showingSummary, let summary = workoutSession.lastSummary {
                WorkoutSummaryView(summary: summary) {
                    showingSummary = false
                }
                .frame(width: geo.size.width, height: geo.size.height)
                .background(Color.black)
            } else {
                BikeComputerView(
                    screenW: geo.size.width,
                    screenH: geo.size.height,
                    onWorkoutEnd: {
                        if workoutSession.lastSummary != nil {
                            showingSummary = true
                        }
                    }
                )
            }
        }
        .ignoresSafeArea()
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
    }
}
