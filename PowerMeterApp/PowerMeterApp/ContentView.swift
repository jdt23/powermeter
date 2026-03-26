import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    var body: some View {
        GeometryReader { geo in
            let topInset = geo.safeAreaInsets.top
            let bottomInset = geo.safeAreaInsets.bottom
            let fullW = geo.size.width
            let fullH = geo.size.height + topInset + bottomInset

            Group {
                if showingSummary, let summary = workoutSession.lastSummary {
                    WorkoutSummaryView(summary: summary) {
                        showingSummary = false
                    }
                    .frame(width: fullW, height: fullH)
                    .background(Color.black)
                } else {
                    BikeComputerView(
                        screenW: fullW,
                        screenH: fullH,
                        onWorkoutEnd: {
                            if workoutSession.lastSummary != nil {
                                showingSummary = true
                            }
                        }
                    )
                }
            }
            .offset(y: -topInset)
        }
        .ignoresSafeArea()
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
    }
}
