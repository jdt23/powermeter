import Foundation
import WatchConnectivity

class PhoneConnectivityManager: NSObject, ObservableObject {
    @Published var isWatchReachable = false
    @Published var watchHeartRate: Double = 0
    @Published var watchSavedWorkout = false

    override init() {
        super.init()
        if WCSession.isSupported() {
            WCSession.default.delegate = self
            WCSession.default.activate()
        }
    }

    func activateWatch() {
        guard WCSession.default.activationState == .activated, WCSession.default.isReachable else { return }
        WCSession.default.sendMessage(["action": "wake"], replyHandler: nil, errorHandler: nil)
    }

    func sendWorkoutAction(_ action: String) {
        guard WCSession.default.isReachable else { return }
        WCSession.default.sendMessage(["action": action], replyHandler: nil, errorHandler: nil)
    }

    func sendMetrics(power: Int, cadence: Int, resistance: Int,
                      speed: Double = 0, distance: Double = 0, calories: Double = 0) {
        guard WCSession.default.isReachable else { return }
        WCSession.default.sendMessage(
            ["power": power, "cadence": cadence, "resistance": resistance,
             "speed": speed, "distance": distance, "calories": calories],
            replyHandler: nil,
            errorHandler: nil
        )
    }
}

extension PhoneConnectivityManager: WCSessionDelegate {
    func session(_ session: WCSession, activationDidCompleteWith activationState: WCSessionActivationState, error: Error?) {
        DispatchQueue.main.async {
            self.isWatchReachable = session.isReachable
            if session.isReachable { self.activateWatch() }
        }
    }

    func sessionDidBecomeInactive(_ session: WCSession) {}

    func sessionDidDeactivate(_ session: WCSession) {
        WCSession.default.activate()
    }

    func sessionReachabilityDidChange(_ session: WCSession) {
        DispatchQueue.main.async {
            self.isWatchReachable = session.isReachable
            if session.isReachable { self.activateWatch() }
        }
    }

    func session(_ session: WCSession, didReceiveMessage message: [String: Any]) {
        if let hr = message["heartRate"] as? Double {
            DispatchQueue.main.async { self.watchHeartRate = hr }
        }
        if message["workoutSaved"] as? Bool == true {
            DispatchQueue.main.async { self.watchSavedWorkout = true }
        }
    }
}
