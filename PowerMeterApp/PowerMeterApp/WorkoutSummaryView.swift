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
        VStack(spacing: 12) {
            Text("Workout Complete")
                .font(.title2)
                .fontWeight(.bold)
                .foregroundColor(.white)
                .padding(.top, 8)

            // Stats grid - compact 2-column layout
            LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible())], spacing: 8) {
                statCell("Duration", formatDuration(summary.duration), .white)
                statCell("Calories", "\(Int(summary.totalCalories)) kcal", .yellow)
                statCell("Avg Power", "\(Int(summary.averagePower)) W", .green)
                statCell("Max Power", "\(Int(summary.maxPower)) W", .green)
                statCell("Avg Cadence", "\(Int(summary.averageCadence)) RPM", .cyan)
                if summary.averageHeartRate > 0 {
                    statCell("Avg HR", "\(Int(summary.averageHeartRate)) BPM", .red)
                }
                if summary.maxHeartRate > 0 {
                    statCell("Max HR", "\(Int(summary.maxHeartRate)) BPM", .red)
                }
            }
            .padding(.horizontal, 16)

            // Apple Health save section
            VStack(spacing: 6) {
                HStack(spacing: 16) {
                    healthDetail("Power", "\(summary.powerSampleCount) pts")
                    healthDetail("Cadence", "\(summary.cadenceSampleCount) pts")
                    healthDetail("HR", "\(summary.heartRateSampleCount) pts")
                }
                .padding(.vertical, 8)
                .padding(.horizontal, 12)
                .background(RoundedRectangle(cornerRadius: 8).fill(Color.white.opacity(0.03)))

                switch saveState {
                case .unsaved:
                    Button(action: saveToHealth) {
                        Label("Save to Apple Health", systemImage: "heart.fill")
                            .font(.subheadline.weight(.semibold))
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 12)
                            .background(Color.red)
                            .foregroundColor(.white)
                            .cornerRadius(10)
                    }
                case .saving:
                    HStack(spacing: 6) {
                        ProgressView().tint(.white)
                        Text("Saving...").font(.caption).foregroundColor(.gray)
                    }.padding(.vertical, 8)
                case .saved:
                    Label("Saved to Apple Health", systemImage: "checkmark.circle.fill")
                        .font(.caption).foregroundColor(.green).padding(.vertical, 8)
                case .failed:
                    VStack(spacing: 4) {
                        Label("Failed to save", systemImage: "exclamationmark.circle.fill")
                            .font(.caption).foregroundColor(.orange)
                        if let errorMessage {
                            Text(errorMessage).font(.caption2).foregroundColor(.gray)
                        }
                        Button("Retry", action: saveToHealth)
                            .font(.caption).padding(.horizontal, 16).padding(.vertical, 6)
                            .background(Color.red).foregroundColor(.white).cornerRadius(6)
                    }.padding(.vertical, 4)
                }
            }
            .padding(.horizontal, 16)

            Button(action: onDismiss) {
                Text("Done")
                    .font(.headline)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 14)
                    .background(Color.blue)
                    .foregroundColor(.white)
                    .cornerRadius(12)
            }
            .padding(.horizontal, 16)
            .padding(.bottom, 12)
        }
    }

    private func statCell(_ label: String, _ value: String, _ color: Color) -> some View {
        VStack(spacing: 2) {
            Text(label)
                .font(.system(size: 10, weight: .medium))
                .foregroundColor(color.opacity(0.7))
                .tracking(0.5)
            Text(value)
                .font(.system(size: 20, weight: .bold, design: .rounded))
                .foregroundColor(.white)
                .minimumScaleFactor(0.7)
                .lineLimit(1)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 10)
        .background(RoundedRectangle(cornerRadius: 10).fill(Color.white.opacity(0.05)))
    }

    private func healthDetail(_ label: String, _ value: String) -> some View {
        VStack(spacing: 1) {
            Text(label).font(.system(size: 9)).foregroundColor(.gray)
            Text(value).font(.system(size: 11, weight: .medium)).foregroundColor(.white)
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
                    errorMessage = error?.localizedDescription ?? "Check Health permissions in Settings."
                }
            }
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
