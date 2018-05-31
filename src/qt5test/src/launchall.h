#ifndef LAUNCHALL_H
#define LAUNCHALL_H
#include<QThread>

class LaunchAll: public QThread {
  Q_OBJECT
public:
  LaunchAll();
  void run();
  void startLaunch();
  void stopLaunch();
  void setStop(){stop = true;}
 private:
  bool  stop;
};

#endif // LAUNCHALL_H
