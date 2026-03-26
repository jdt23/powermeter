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
                    disableSafeArea()
                }
                .preferredColorScheme(.dark)
        }
    }

    /// Disable safe area on the ROOT hosting controller that SwiftUI creates.
    /// This is the only way to get true edge-to-edge content.
    private func disableSafeArea() {
        DispatchQueue.main.async {
            guard let scene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
                  let window = scene.windows.first,
                  let rootVC = window.rootViewController else { return }

            // Cancel out the system safe area insets with negative additionalSafeAreaInsets
            let insets = rootVC.view.safeAreaInsets
            rootVC.additionalSafeAreaInsets = UIEdgeInsets(
                top: -insets.top,
                left: -insets.left,
                bottom: -insets.bottom,
                right: -insets.right
            )
        }
    }
}
