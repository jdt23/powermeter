import SwiftUI

struct WorkoutSummaryView: View {
    @EnvironmentObject var healthKitManager: HealthKitManager
    @EnvironmentObject var workoutSession: WorkoutSession

    let summary: WorkoutSummary
    let onDismiss: () -> Void

    @State private var saveState: SaveState = .unsaved
    @State private var errorMessage: String?

    enum SaveState {
        case unsaved
        case saving
        case saved
        case failed
    }

    var body: some View {
        ScrollView {
            VStack(spacing: 24) {
                Text("Workout Complete")
                    .font(.largeTitle)
                    .fontWeight(.bold)
                    .foregroundColor(.white)

                // Stats summary
                VStack(spacing: 16) {
                    summaryRow("Duration", value: formatDuration(summary.duration))
                    Divider().background(Color.gray)
                    summaryRow("Avg Power", value: "\(Int(summary.averagePower)) W")
                    summaryRow("Max Power", value: "\(Int(summary.maxPower)) W")
                    Divider().background(Color.gray)
                    summaryRow("Avg Cadence", value: "\(Int(summary.averageCadence)) RPM")
                    Divider().background(Color.gray)
                    if summary.averageHeartRate > 0 {
                        summaryRow("Avg Heart Rate", value: "\(Int(summary.averageHeartRate)) BPM")
                        summaryRow("Max Heart Rate", value: "\(Int(summary.maxHeartRate)) BPM")
                        Divider().background(Color.gray)
                    }
                    summaryRow("Calories", value: "\(Int(summary.totalCalories)) kcal")
                }
                .padding(24)
                .background(
                    RoundedRectangle(cornerRadius: 16)
                        .fill(Color.white.opacity(0.05))
                )

                // Apple Health section
                VStack(spacing: 12) {
                    Text("Apple Health Data")
                        .font(.headline)
                        .foregroundColor(.white)

                    VStack(spacing: 8) {
                        healthRow("Workout Type", value: "Indoor Cycling")
                        healthRow("Power Samples", value: "\(summary.powerSampleCount)")
                        healthRow("Cadence Samples", value: "\(summary.cadenceSampleCount)")
                        healthRow("Heart Rate Samples", value: "\(summary.heartRateSampleCount)")
                        healthRow("Active Energy", value: "\(Int(summary.totalCalories)) kcal")
                    }
                    .padding(16)
                    .background(
                        RoundedRectangle(cornerRadius: 12)
                            .fill(Color.white.opacity(0.03))
                    )

                    switch saveState {
                    case .unsaved:
                        Button(action: saveToHealth) {
                            Label("Save to Apple Health", systemImage: "heart.fill")
                                .font(.headline)
                                .frame(maxWidth: .infinity)
                                .padding(.vertical, 16)
                                .background(Color.red)
                                .foregroundColor(.white)
                                .cornerRadius(12)
                        }

                    case .saving:
                        HStack(spacing: 8) {
                            ProgressView().tint(.white)
                            Text("Saving...")
                                .foregroundColor(.gray)
                        }
                        .padding(.vertical, 12)

                    case .saved:
                        Label("Saved to Apple Health", systemImage: "checkmark.circle.fill")
                            .font(.subheadline)
                            .foregroundColor(.green)
                            .padding(.vertical, 12)

                    case .failed:
                        VStack(spacing: 8) {
                            Label("Failed to save", systemImage: "exclamationmark.circle.fill")
                                .font(.subheadline)
                                .foregroundColor(.orange)
                            if let errorMessage {
                                Text(errorMessage)
                                    .font(.caption)
                                    .foregroundColor(.gray)
                                    .multilineTextAlignment(.center)
                            }
                            Button(action: saveToHealth) {
                                Text("Retry")
                                    .font(.subheadline)
                                    .padding(.horizontal, 24)
                                    .padding(.vertical, 10)
                                    .background(Color.red)
                                    .foregroundColor(.white)
                                    .cornerRadius(8)
                            }
                        }
                        .padding(.vertical, 8)
                    }
                }

                Button(action: onDismiss) {
                    Text("Done")
                        .font(.headline)
                        .frame(maxWidth: .infinity)
                        .padding(.vertical, 16)
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(12)
                }
            }
            .padding(24)
        }
    }

    private func saveToHealth() {
        saveState = .saving
        errorMessage = nil

        healthKitManager.saveWorkout(
            start: summary.startDate,
            end: summary.endDate,
            powerSamples: workoutSession.powerSamples,
            cadenceSamples: workoutSession.cadenceSamples,
            heartRateSamples: workoutSession.heartRateSamples,
            totalCalories: summary.totalCalories
        ) { success, error in
            DispatchQueue.main.async {
                if success {
                    saveState = .saved
                } else {
                    saveState = .failed
                    errorMessage = error?.localizedDescription ?? "Unknown error. Check Health permissions in Settings."
                }
            }
        }
    }

    private func summaryRow(_ label: String, value: String) -> some View {
        HStack {
            Text(label)
                .foregroundColor(.gray)
            Spacer()
            Text(value)
                .fontWeight(.semibold)
                .foregroundColor(.white)
        }
    }

    private func healthRow(_ label: String, value: String) -> some View {
        HStack {
            Text(label)
                .font(.caption)
                .foregroundColor(.gray)
            Spacer()
            Text(value)
                .font(.caption)
                .fontWeight(.medium)
                .foregroundColor(.white)
        }
    }

    private func formatDuration(_ interval: TimeInterval) -> String {
        let hours = Int(interval) / 3600
        let minutes = (Int(interval) % 3600) / 60
        let seconds = Int(interval) % 60
        if hours > 0 {
            return String(format: "%d:%02d:%02d", hours, minutes, seconds)
        }
        return String(format: "%02d:%02d", minutes, seconds)
    }
}
