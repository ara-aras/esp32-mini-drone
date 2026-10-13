#pragma once

// HTTP dashboard on the drone's access point. Call after netBegin().
void webBegin();
void webLoop();   // call often from loop()
