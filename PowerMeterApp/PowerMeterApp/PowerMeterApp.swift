import UIKit
import SwiftUI

// Use UIKit app lifecycle instead of SwiftUI App protocol.
// This is the ONLY way to set safeAreaRegions=[] on the ROOT
// hosting controller, which is required for true edge-to-edge.

@main
class AppDelegate: UIResponder, UIApplicationDelegate {
    func application(
        _ application: UIApplication,
        didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
    ) -> Bool {
        return true
    }

    func application(
        _ application: UIApplication,
        configurationForConnecting connectingSceneSession: UISceneSession,
        options: UIScene.ConnectionOptions
    ) -> UISceneConfiguration {
        let config = UISceneConfiguration(name: nil, sessionRole: connectingSceneSession.role)
        config.delegateClass = SceneDelegate.self
        return config
    }
}

class SceneDelegate: UIResponder, UIWindowSceneDelegate {
    var window: UIWindow?

    // State objects live here — they persist for the app's lifetime
    let bleManager = BLEManager()
    let healthKitManager = HealthKitManager()
    let workoutSession = WorkoutSession()
    let connectivityManager = PhoneConnectivityManager()

    func scene(
        _ scene: UIScene,
        willConnectTo session: UISceneSession,
        options connectionOptions: UIScene.ConnectionOptions
    ) {
        guard let windowScene = scene as? UIWindowScene else { return }

        healthKitManager.requestAuthorization()

        let rootView = ContentView()
            .environmentObject(bleManager)
            .environmentObject(healthKitManager)
            .environmentObject(workoutSession)
            .environmentObject(connectivityManager)
            .preferredColorScheme(.dark)

        let hostingController = UIHostingController(rootView: rootView)
        hostingController.safeAreaRegions = []
        hostingController.view.backgroundColor = .black

        let window = UIWindow(windowScene: windowScene)
        window.rootViewController = hostingController
        window.makeKeyAndVisible()
        self.window = window
    }
}
