#include "chase.hpp"
#include "grid.hpp"
#include <cstdio>
#include <raylib.h>

int main()
{
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 600;
    const int CELL_SIZE = 15;

    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "City Chase");

    setbuf(stdout, NULL);

    Grid grid(WINDOW_WIDTH, WINDOW_HEIGHT, CELL_SIZE);
    Chase chase(WINDOW_WIDTH, WINDOW_HEIGHT, CELL_SIZE, grid);

    SetTargetFPS(60);

    grid.generateCity(2);
    chase.reset();

    int restartTimer = 0; // countdown for the auto restart (420 = 7 sec at 60 fps)

    while (!WindowShouldClose()) {
        // R starts a new city (full reset, including the blue speed bonus).
        if (IsKeyPressed(KEY_R)) {
            grid.generateCity(2);
            chase.reset(true);
            restartTimer = 0;
        }

        // SPACE restarts the chase (keeps the blue speed bonus).
        if (IsKeyPressed(KEY_SPACE)) {
            chase.reset(false);
            restartTimer = 0;
        }

        // Auto restart after 7 seconds once the chase is over.
        if (chase.isFinished()) {
            restartTimer++;
            if (restartTimer >= 420) {
                chase.reset(false);
                restartTimer = 0;
            }
        }
        else {
            restartTimer = 0;
        }

        // Run the chase.
        chase.update();

        // Draw.
        BeginDrawing();
        ClearBackground(DARKGRAY);

        grid.draw();
        chase.draw();

        // HUD: blue speed bonus.
        int bonus = chase.getBlueExtraMoves();
        if (bonus > 0) {
            char text[64];
            snprintf(text, sizeof(text), "Blue speed: +%d", bonus);
            DrawText(text, 10, 10, 16, BLUE);
        }

        // Game over screen.
        if (chase.isFinished()) {
            // Darken the background.
            DrawRectangle(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, Color{0, 0, 0, 160});
            DrawRectangle(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, Color{0, 0, 0, 40}); // soft gradient

            // Panel size.
            int panelW = 420, panelH = 270;
            int px = (WINDOW_WIDTH - panelW) / 2;
            int py = (WINDOW_HEIGHT - panelH) / 2;

            // Shadow under the panel.
            DrawRectangle(px + 4, py + 4, panelW, panelH, Color{0, 0, 0, 100});

            // Main panel background.
            DrawRectangle(px, py, panelW, panelH, Color{20, 22, 30, 245});

            // Outer frame (main).
            Color borderColor = chase.isCrashed() ? ORANGE : RED;
            Color borderGlow = chase.isCrashed() ? Color{255, 140, 0, 60} : Color{200, 30, 30, 60};
            DrawRectangleLines(px, py, panelW, panelH, borderColor);
            DrawRectangleLines(px + 1, py + 1, panelW - 2, panelH - 2, borderGlow);
            DrawRectangleLines(px + 3, py + 3, panelW - 6, panelH - 6, Color{255, 255, 255, 20});

            // Top accent bar.
            DrawRectangle(px + 10, py, panelW - 20, 3, borderColor);

            // Status label.
            const char *statusLabel = chase.isCrashed() ? "CRASH" : "CATCH!";
            Color statusColor = chase.isCrashed() ? ORANGE : RED;
            Color statusBg = chase.isCrashed() ? Color{180, 80, 0, 30} : Color{180, 0, 0, 30};

            int statusSize = 16;
            int statusW = MeasureText(statusLabel, statusSize);
            DrawRectangle(px + (panelW - statusW) / 2 - 12, py + 18, statusW + 24, 26, statusBg);
            DrawRectangleLines(px + (panelW - statusW) / 2 - 12, py + 18, statusW + 24, 26, Color{255, 255, 255, 30});
            DrawText(statusLabel, px + (panelW - statusW) / 2, py + 24, statusSize, statusColor);

            // Title.
            const char *title = chase.isCrashed() ? "The driver crashed!" : "The police caught the thief!";
            Color titleColor = chase.isCrashed() ? ORANGE : RED;
            int titleSize = 36;
            int titleW = MeasureText(title, titleSize);
            // If the title is wider than the panel - shrink the font.
            if (titleW > panelW - 30) {
                titleSize = 26;
                titleW = MeasureText(title, titleSize);
            }
            DrawText(title, px + (panelW - titleW) / 2, py + 60, titleSize, titleColor);

            // Divider.
            for (int i = 0; i < 30; i++) {
                int x = px + (panelW - 160) / 2 + i * 6;
                Color c = Color{(unsigned char)(chase.isCrashed() ? (200 + i * 2) : (180 + i * 2)),
                                (unsigned char)(chase.isCrashed() ? (90 + i) : 20),
                                (unsigned char)(chase.isCrashed() ? 0 : 20), 180};
                DrawRectangle(x, py + 102, 4, 2, c);
            }

            // Subtitle (details).
            const char *subtitle = chase.isCrashed() ? "Red crashed into a building!" : "Red caught the blue car!";
            int subSize = 18;
            int subW = MeasureText(subtitle, subSize);
            DrawText(subtitle, px + (panelW - subW) / 2, py + 115, subSize, LIGHTGRAY);

            // Auto restart counter (big).
            int secLeft = 7 - restartTimer / 60;
            if (secLeft < 0)
                secLeft = 0;

            char timerText[32];
            snprintf(timerText, sizeof(timerText), "Auto-restart in %ds", secLeft);
            int timerSize = 24;
            int timerW = MeasureText(timerText, timerSize);

            // Progress bar under the timer.
            int barW = 200, barH = 6;
            int barX = px + (panelW - barW) / 2;
            int barY = py + 170;
            float progress = restartTimer / 420.0f;
            if (progress > 1.0f)
                progress = 1.0f;
            DrawRectangle(barX, barY, barW, barH, Color{60, 60, 70, 200});
            DrawRectangle(barX, barY, (int)(barW * progress), barH, chase.isCrashed() ? ORANGE : RED);
            DrawRectangleLines(barX, barY, barW, barH, Color{100, 100, 110, 80});

            // Timer text.
            DrawText(timerText, px + (panelW - timerW) / 2, py + 140, timerSize, WHITE);

            // Instructions.
            char instruction[] = "[SPACE] restart    [R] new city";
            int instructionSize = 16;
            int instructionW = MeasureText(instruction, instructionSize);
            DrawText(instruction, px + (panelW - instructionW) / 2, py + 192, instructionSize,
                     Color{140, 140, 160, 200});

            // Bonus on crash.
            if (chase.isCrashed()) {
                char bonusText[64];
                snprintf(bonusText, sizeof(bonusText), "Blue gets +%d speed for the next round!",
                         chase.getBlueExtraMoves());
                int bonusSize = 16;
                int bonusW = MeasureText(bonusText, bonusSize);
                if (bonusW > panelW - 20) {
                    bonusSize = 14;
                    bonusW = MeasureText(bonusText, bonusSize);
                }
                DrawText(bonusText, px + (panelW - bonusW) / 2, py + 222, bonusSize, Color{80, 140, 255, 220});
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
