import SwiftUI

@main
struct PowerMeterApp: App {
    @StateObject private var bleManager = BLEManager()
    @StateObject private var healthKitManager = HealthKitManager()
    @StateObject private var workoutSession = WorkoutSession()
    @StateObject private var connectivityManager = PhoneConnectivityManager()

    var body: some Scene {
        WindowGroup {
            FullScreenContainer {
                ContentView()
            }
            .ignoresSafeArea()
            .environmentObject(bleManager)
            .environmentObject(healthKitManager)
            .environmentObject(workoutSession)
            .environmentObject(connectivityManager)
            .onAppear {
                healthKitManager.requestAuthorization()
            }
            .preferredColorScheme(.dark)
            .statusBarHidden(true)
            .persistentSystemOverlays(.hidden)
        }
    }
}

/// Wraps content in a UIHostingController with safeAreaRegions = []
/// This is the only reliable way to disable safe areas on iOS 26.
struct FullScreenContainer<Content: View>: UIViewControllerRepresentable {
    @ViewBuilder let content: Content

    func makeUIViewController(context: Context) -> UIHostingController<Content> {
        let vc = UIHostingController(rootView: content)
        vc.safeAreaRegions = []
        vc.view.backgroundColor = .black
        return vc
    }

    func updateUIViewController(_ vc: UIHostingController<Content>, context: Context) {
        vc.rootView = content
        vc.safeAreaRegions = []
    }
}
