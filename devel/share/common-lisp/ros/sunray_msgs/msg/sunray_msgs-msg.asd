
(cl:in-package :asdf)

(defsystem "sunray_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "agent_state" :depends-on ("_package_agent_state"))
    (:file "_package_agent_state" :depends-on ("_package"))
    (:file "orca_cmd" :depends-on ("_package_orca_cmd"))
    (:file "_package_orca_cmd" :depends-on ("_package"))
    (:file "orca_state" :depends-on ("_package_orca_state"))
    (:file "_package_orca_state" :depends-on ("_package"))
    (:file "rmtt_cmd" :depends-on ("_package_rmtt_cmd"))
    (:file "_package_rmtt_cmd" :depends-on ("_package"))
    (:file "rmtt_orca" :depends-on ("_package_rmtt_orca"))
    (:file "_package_rmtt_orca" :depends-on ("_package"))
    (:file "rmtt_state" :depends-on ("_package_rmtt_state"))
    (:file "_package_rmtt_state" :depends-on ("_package"))
    (:file "station_cmd" :depends-on ("_package_station_cmd"))
    (:file "_package_station_cmd" :depends-on ("_package"))
    (:file "ugv_cmd" :depends-on ("_package_ugv_cmd"))
    (:file "_package_ugv_cmd" :depends-on ("_package"))
  ))