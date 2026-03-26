import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession
    @State private var showingSummary = false

    // Get real values from UIKit — SwiftUI's safe area APIs lie when ignoresSafeArea is used
    private var fullW: CGFloat { UIScreen.main.bounds.width }
    private var fullH: CGFloat { UIScreen.main.bounds.height }
    private var topInset: CGFloat {
        UIApplication.shared.connectedScenes
            .compactMap { $0 as? UIWindowScene }
            .first?.windows.first?.safeAreaInsets.top ?? 0
    }

    var body: some View {
        ZStack {
            Color.black

            if showingSummary, let summary = workoutSession.lastSummary {
                WorkoutSummaryView(summary: summary) {
                    showingSummary = false
                }
                .frame(width: fullW, height: fullH)
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
        .frame(width: fullW, height: fullH)
        .position(x: fullW / 2, y: fullH / 2)
        .ignoresSafeArea()
        .statusBarHidden(true)
        .persistentSystemOverlays(.hidden)
    }
}
