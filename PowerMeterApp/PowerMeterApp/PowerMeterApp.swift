import SwiftUI

@main
struct PowerMeterApp: App {
    @UIApplicationDelegateAdaptor(AppDelegate.self) var appDelegate

    var body: some Scene {
        WindowGroup {
            // This is unused — SceneDelegate sets up the real root view
            Color.black
        }
    }
}

// MARK: - UIKit App/Scene Delegates for full-screen control

class AppDelegate: NSObject, UIApplicationDelegate {
    // Shared state objects — created once, shared via SceneDelegate
    let bleManager = BLEManager()
    let healthKitManager = HealthKitManager()
    let workoutSession = WorkoutSession()
    let connectivityManager = PhoneConnectivityManager()

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

class SceneDelegate: NSObject, UIWindowSceneDelegate {
    var window: UIWindow?

    func scene(_ scene: UIScene, willConnectTo session: UISceneSession,
               options connectionOptions: UIScene.ConnectionOptions) {
        guard let windowScene = scene as? UIWindowScene else { return }

        let appDelegate = UIApplication.shared.delegate as! AppDelegate
        appDelegate.healthKitManager.requestAuthorization()

        let contentView = ContentView()
            .environmentObject(appDelegate.bleManager)
            .environmentObject(appDelegate.healthKitManager)
            .environmentObject(appDelegate.workoutSession)
            .environmentObject(appDelegate.connectivityManager)
            .preferredColorScheme(.dark)

        let hostingController = UIHostingController(rootView: contentView)
        hostingController.safeAreaRegions = []           // DISABLE safe area entirely
        hostingController.view.backgroundColor = .black

        let window = UIWindow(windowScene: windowScene)
        window.rootViewController = hostingController
        window.makeKeyAndVisible()
        self.window = window
    }
}
