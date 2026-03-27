import UIKit
import SwiftUI

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

/// Custom UIHostingController that dynamically negates safe area insets.
/// safeAreaRegions = [] is broken on iOS 26 — this is the workaround.
class FullScreenHostingController<Content: View>: UIHostingController<Content> {
    private var didApplyInsets = false

    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        if !didApplyInsets {
            didApplyInsets = true
            let insets = view.safeAreaInsets
            if insets.top > 0 || insets.bottom > 0 {
                additionalSafeAreaInsets = UIEdgeInsets(
                    top: -insets.top,
                    left: -insets.left,
                    bottom: -insets.bottom,
                    right: -insets.right
                )
            }
        }
    }

    override var prefersStatusBarHidden: Bool { true }
    override var prefersHomeIndicatorAutoHidden: Bool { true }
    override var preferredScreenEdgesDeferringSystemGestures: UIRectEdge { .all }
}

class SceneDelegate: UIResponder, UIWindowSceneDelegate {
    var window: UIWindow?

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

        let hostingController = FullScreenHostingController(rootView: rootView)
        hostingController.safeAreaRegions = []
        hostingController.view.backgroundColor = .black

        let window = UIWindow(windowScene: windowScene)
        window.rootViewController = hostingController
        window.makeKeyAndVisible()
        self.window = window

        // Wake the Watch app immediately
        connectivityManager.activateWatch()
    }
}
