///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#pragma once


#include <list>
#include <map>
#include <memory>
#include <typeinfo>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/interface_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/controller_info.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace hardware_interface
{

/** \brief Robot Hardware Interface and Resource Manager
 *　このクラスは、コントローラマネージャにロボットのハードウェアインタフェースのセットへの標準化されたインタフェースを提供します。
  与えられたコントローラのセットに対してリソースの競合チェックを行い、ハードウェアインタフェースのマップを保持します。
 　カスタムロボットハードウェアを抽象化するためのベースクラスとして使用されます。

 *　ハードウェアインターフェースマップ ( interfaces_) は、
    HardwareInterface から派生したインターフェースタイプの名前と、
    それらのインターフェースタイプのインスタンスとの間の 1 対 1 のマップです。
 * 
 * この基本インターフェースから派生したクラスはロボットを表し、
 * ロボットのハードウェアリソース（関節、センサー、アクチュエータ）の状態と
 * 送信コマンドをデータメンバの配列に格納します。
 * これらのメンバの名前は、意味的な意味を持つものでなければなりません（例：pos, vel, eff, cmd）。
 * コントローラからのコマンドは、
 * コントローラの更新メソッド (controller_interface::ControllerBase::update()) によって生成されます。
 *  
 * 各リソースに対して、JointStateHandle（読み取り専用ジョイント用）、
 * JointHandle（読み取りおよび書き込みジョイント用）、
 * またはカスタムハンドルを使用して、
 * ロボットのインターフェースタイプのいずれかで登録された
 * （hardware_interface::ResourceManager::registerHandle）ハンドルを使用することができます。
 * ジョイントからの読み込みのみが必要な場合には、コントローラ間の競合が発生しないため、
 * JointStateHandle を使用することが望ましいことに注意してください。
 * 
 *  読み取り専用のジョイントにはJointStateInterfaceを使用することができ、
 * コマンドを受け入れてフィードバック（読み書き）を提供するジョイントには
 * JointCommandInterfaceまたはその派生インターフェースの1つ（例：PositionJointInterface）を使用することができます。
 *  もう一つのオプションは、カスタムのものを定義して使用することです。 
 * インターフェースは、派生したロボットクラスに登録（registerInterface）されます。
 * インターフェースの登録(registerInterface)は、
 * カスタムロボットハードウェアクラスのコンストラクタまたはinitで行うことができます。
 */
class RobotHW : public InterfaceManager
{
public:
  virtual ~RobotHW() = default;

  /** \brief 非リアルタイムスレッドからRobotHWを初期化するためにinit関数を呼び出します。
    カスタムロボットの初期化は、ジョイントハンドルを登録することで行います。
    (ref hardware_interface::ResourceManager::registerHandle) to hardware
    インターフェイスをグループ化し、それらの個々のハードウェア・インターフェイスをカスタム・ロボットを表すクラスに登録します。
    (この hardware_interface::RobotHW から派生)
   
    \note ジョイントハンドルとインターフェースの登録は、コンストラクタで行うか、この  init メソッドで行うことができます。
   
    \param root_nh 呼び出し元の名前空間のルートにある NodeHandle
   
    \param robot_hw_nh RobotHWがその設定を読み込むべきネームスペースのNodeHandle
   
    \returns 初期化が成功したらTrueを返す
   */
  virtual bool init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle &/*robot_hw_nh*/) {return true;}

  /** \name Resource Management
   *\{*/

  /** (非リアルタイムで) 与えられたコントローラのセットが同時に実行できるかどうかをチェックします。　
このデフォルトの実装では、2つのコントローラが同じリソースを使用しているかどうかをチェックします。
   */
  virtual bool checkForConflict(const std::list<ControllerInfo>& info) const
  {
    // Map from resource name to all controllers claiming it
    std::map<std::string, std::list<ControllerInfo>> resource_map;

    // Populate a map of all controllers claiming individual resources.
    // We do this by iterating over every claimed resource of every hardware interface used by every controller
    for (const auto& controller_info : info)
    {
      for (const auto& claimed_resource : controller_info.claimed_resources)
      {
        for (const auto& iface_resource : claimed_resource.resources)
        {
          resource_map[iface_resource].push_back(controller_info);
        }
      }
    }

    // Enforce resource exclusivity policy: No resource can be claimed by more than one controller
    bool in_conflict = false;
    for (const auto& resource_name_and_claiming_controllers : resource_map)
    {
      if (resource_name_and_claiming_controllers.second.size() > 1)
      {
        std::string controller_list;
        for (const auto& controller : resource_name_and_claiming_controllers.second)
          controller_list += controller.name + ", ";
        ROS_WARN("Resource conflict on [%s].  Controllers = [%s]", resource_name_and_claiming_controllers.first.c_str(), controller_list.c_str());
        in_conflict = true;
      }
    }

    return in_conflict;
  }
  /**\}*/

  /** \name Hardware Interface Switching
   *\{*/

  /**　必要なハードウェアインターフェイススイッチについて、RobotHWの現在の状態から、与えられたコントローラが起動・停止できるかどうかを（非リアルタイムで）確認し、切り替えの準備をします。起動リストと停止リストは分離されています。　
      これはチェックと準備を処理し、実際のスイッチはdoSwitch()でコミットされます。
   */
  virtual bool prepareSwitch(const std::list<ControllerInfo>& /*start_list*/,
                             const std::list<ControllerInfo>& /*stop_list*/) { return true; }

  /**
   * 与えられたコントローラを起動したり停止したりするために必要なすべてのハードウェアインターフェイススイッチを (リアルタイムで) 実行します。
    開始リストと停止リストは不連続です。実現可能性はprepareSwitch()で事前にチェックしています。
   */
  virtual void doSwitch(const std::list<ControllerInfo>& /*start_list*/,
                        const std::list<ControllerInfo>& /*stop_list*/) {}

  enum class SwitchState
  {
    DONE,
    ONGOING,
    ERROR
  };

  /** \brief Return (in realtime) the state of the last doSwitch(). */
  virtual SwitchState switchResult() const
  {
    return SwitchState::DONE;
  }

  /** \brief Return (in realtime) the state of the last doSwitch() for a given controller. */
  virtual SwitchState switchResult(const ControllerInfo& /*controller*/) const
  {
    return SwitchState::DONE;
  }
  /**\}*/

  /** \name Control Loop
   *\{*/

  /** \brief Read data from the robot hardware.
   *readメソッドは制御ループサイクル（read, update, write）の一部であり、ロボットのハードウェアリソース（関節、センサ、アクチュエータ）からロボットの状態を入力するために使用されます
    このメソッドは、controller_manager::ControllerManager::update() と write の前に呼び出す必要があります。

   * \note readとは、ハードウェアからの読み込み状態を指します。
      これは、ハードウェアにコマンドを書き込むことを指す write を補完するものです。
   *
   * 読み込み中のWallTimeの問い合わせはリアルタイムセーフではありません。
   * パラメータtimeとperiodを使用することで、リアルタイムソースから時間を注入することが可能になります。
   *
   * \param time 現在の時刻
   * \param period 最後に呼び出されてからの経過時間
   */
  virtual void read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {}

  /** \brief Write commands to the robot hardware.
   * writeメソッドは、制御ループサイクル（read, update, write）の一部であり、ロボットのハードウェアリソース（ジョイント、アクチュエータ）にコマンドを送信するために使用されます。
    このメソッドは、readとcontroller_manager::ControllerManager::updateの後に呼び出す必要があります。

   * \note 書き込みとは、ハードウェアにコマンドを書き込むことを指します。
   * これは、ハードウェアからの読み出し状態を指すreadを補完するものです。
   *
    書き込みの内部でWallTimeをクエリすることは、リアルタイムセーフではありません。
    timeとperiodというパラメータを使うと、リアルタイムソースから時間を注入することが可能になります。   *
   * \param time 現在の時刻
   * \param period 最後の書き込み呼び出しからの経過時間
   */
  virtual void write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {}

  /**\}*/
};

typedef std::shared_ptr<RobotHW> RobotHWSharedPtr;

}
