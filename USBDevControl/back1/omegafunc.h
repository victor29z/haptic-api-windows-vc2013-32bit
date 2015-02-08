// #pragma comment(lib£¬"omegafunc.lib")
extern "C" void __declspec(dllexport) Force2torque(double F[3], double T[3], double HX[3], double C[4]);
extern "C" void __declspec(dllexport) Encoder2handle(double E[3], double HX[3], double C[4]);