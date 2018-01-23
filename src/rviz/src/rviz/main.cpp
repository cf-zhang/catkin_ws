/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QApplication>

#include "rviz/visualizer_app.h"

#include "logindlg.h"
int main( int argc, char** argv )
{
  QApplication qapp( argc, argv );
  LoginDlg login;
  rviz::VisualizerApp vapp;
  vapp.setApp( &qapp );
  if(login.exec()==QDialog::Accepted)
  {
    if( vapp.init( argc, argv ))
    {
        return qapp.exec();
    }
  }
  else
  {
    return 1;
  }
  //  QApplication qapp( argc, argv );
  //   LoginDlg login;
  //  login.appendAccount("admin","admin");
  //  login.appendAccount("cfzhang","cfzhang");

  //  TRsa rsa = TRsa("/home/cfzhang/catkin_ws/src/rviz/rsakey.pub","/home/cfzhang/catkin_ws/src/rviz/rsakey");
    
  //  login.appendAccount("admin","admin");
  //  login.appendAccount("cfzhang","cfzhang");
    // TRsa rsa;
    // rsa.rsa_generate_key("/home/cfzhang/catkin_ws/src/rviz/rsakey");
  //  std::string src="hello World";
  //  std::string dest="";
  //  std::string ddest;
  //  rsa.public_key_encrypt(src, dest);
  //  printf("%s\n", dest.c_str());
  //  rsa.private_key_decrypt(dest, ddest);
  //  printf("%s\n", ddest.c_str());


}
