import Foundation
import WatchConnectivity

class WatchConnectivityManager: NSObject, ObservableObject {
    @Published var isPhoneReachable = false

    var workoutManager: WatchWorkoutManager?

    override init() {
        super.init()
        if WCSession.isSupported() {
            WCSession.default.delegate = self
            WCSession.default.activate()
        }
    }

    func sendHeartRate(_ bpm: Double) {
        guard WCSession.default.isReachable else { return }
        WCSession.default.sendMessage(["heartRate": bpm], replyHandler: nil, errorHandler: nil)
    }
}

extension WatchConnectivityManager: WCSessionDelegate {
    func session(_ session: WCSession, activationDidCompleteWith activationState: WCSessionActivationState, error: Error?) {
        DispatchQueue.main.async {
            self.isPhoneReachable = session.isReachable
        }
    }

    func sessionReachabilityDidChange(_ session: WCSession) {
        DispatchQueue.main.async {
            self.isPhoneReachable = session.isReachable
        }
    }

    func session(_ session: WCSession, didReceiveMessage message: [String: Any]) {
        if let action = message["action"] as? String {
            DispatchQueue.main.async {
                switch action {
                case "start":
                    self.workoutManager?.requestAuthorization()
                    self.workoutManager?.startWorkout()
                case "pause":
                    self.workoutManager?.pauseWorkout()
                case "resume":
                    self.workoutManager?.resumeWorkout()
                case "stop":
                    self.workoutManager?.stopWorkout()
                default:
                    break
                }
            }
        }

        // Receive sensor metrics from iPhone
        if let power = message["power"] as? Int,
           let cadence = message["cadence"] as? Int,
           let resistance = message["resistance"] as? Int {
            let speed = message["speed"] as? Double ?? 0
            let distance = message["distance"] as? Double ?? 0
            let calories = message["calories"] as? Double ?? 0
            self.workoutManager?.updateMetrics(
                power: power, cadence: cadence, resistance: resistance,
                speed: speed, distance: distance, calories: calories)
        }
    }
}
