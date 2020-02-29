Bridge.assembly("BridgeGeneralRoboticsToolbox", function ($asm, globals) {
    "use strict";


    var $m = Bridge.setMetadata,
        $n = ["MathNet.Numerics.LinearAlgebra","TestGeneralRoboticsToolboxNET","System","MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra"];
    $m("TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox", function () { return {"att":1048577,"a":2,"m":[{"a":2,"n":".ctor","t":1,"sn":"ctor"},{"a":2,"n":"Cross","is":true,"t":8,"pi":[{"n":"left","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"right","pt":$n[0].Vector$1(System.Double),"ps":1}],"sn":"Cross","rt":$n[0].Vector$1(System.Double),"p":[$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double)]},{"a":2,"n":"Fwdkin","is":true,"t":8,"pi":[{"n":"robot","pt":$n[1].Robot,"ps":0},{"n":"theta","pt":$n[2].Array.type(System.Double),"ps":1}],"sn":"Fwdkin","rt":$n[1].Transform,"p":[$n[1].Robot,$n[2].Array.type(System.Double)]},{"a":2,"n":"Hat","is":true,"t":8,"pi":[{"n":"k","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Hat","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Invhat","is":true,"t":8,"pi":[{"n":"khat","pt":$n[0].Matrix$1(System.Double),"ps":0}],"sn":"Invhat","rt":$n[0].Vector$1(System.Double),"p":[$n[0].Matrix$1(System.Double)]},{"a":2,"n":"Q2R","is":true,"t":8,"pi":[{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Q2R","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Q2Rot","is":true,"t":8,"pi":[{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Q2Rot","rt":$n[2].Tuple$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double),System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Quatcomplement","is":true,"t":8,"pi":[{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Quatcomplement","rt":$n[0].Vector$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Quatjacobian","is":true,"t":8,"pi":[{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Quatjacobian","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Quatproduct","is":true,"t":8,"pi":[{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Quatproduct","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"R2Q","is":true,"t":8,"pi":[{"n":"R","pt":$n[0].Matrix$1(System.Double),"ps":0}],"sn":"R2Q","rt":$n[0].Vector$1(System.Double),"p":[$n[0].Matrix$1(System.Double)]},{"a":2,"n":"R2Rpy","is":true,"t":8,"pi":[{"n":"R","pt":$n[0].Matrix$1(System.Double),"ps":0}],"sn":"R2Rpy","rt":$n[0].Vector$1(System.Double),"p":[$n[0].Matrix$1(System.Double)]},{"a":2,"n":"R2rot","is":true,"t":8,"pi":[{"n":"R","pt":$n[0].Matrix$1(System.Double),"ps":0}],"sn":"R2rot","rt":$n[2].Tuple$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double),System.Double),"p":[$n[0].Matrix$1(System.Double)]},{"a":2,"n":"Robotjacobian","is":true,"t":8,"pi":[{"n":"robot","pt":$n[1].Robot,"ps":0},{"n":"theta","pt":$n[2].Array.type(System.Double),"ps":1}],"sn":"Robotjacobian","rt":$n[0].Matrix$1(System.Double),"p":[$n[1].Robot,$n[2].Array.type(System.Double)]},{"a":2,"n":"Rot","is":true,"t":8,"pi":[{"n":"k","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"theta","pt":$n[2].Double,"ps":1}],"sn":"Rot","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double),$n[2].Double]},{"a":2,"n":"Rot2Q","is":true,"t":8,"pi":[{"n":"k","pt":$n[2].Array.type(System.Double),"ps":0},{"n":"theta","pt":$n[2].Double,"ps":1}],"sn":"Rot2Q","rt":$n[0].Vector$1(System.Double),"p":[$n[2].Array.type(System.Double),$n[2].Double]},{"a":2,"n":"Rpy2R","is":true,"t":8,"pi":[{"n":"rpy","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Rpy2R","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Screw_matrix","is":true,"t":8,"pi":[{"n":"r","pt":$n[0].Vector$1(System.Double),"ps":0}],"sn":"Screw_matrix","rt":$n[0].Matrix$1(System.Double),"p":[$n[0].Vector$1(System.Double)]},{"a":2,"n":"Subproblem0","is":true,"t":8,"pi":[{"n":"p","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":1},{"n":"k","pt":$n[0].Vector$1(System.Double),"ps":2}],"sn":"Subproblem0","rt":$n[2].Double,"p":[$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double)],"box":function ($v) { return Bridge.box($v, System.Double, System.Double.format, System.Double.getHashCode);}},{"a":2,"n":"Subproblem1","is":true,"t":8,"pi":[{"n":"p","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":1},{"n":"k","pt":$n[0].Vector$1(System.Double),"ps":2}],"sn":"Subproblem1","rt":$n[2].Double,"p":[$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double)],"box":function ($v) { return Bridge.box($v, System.Double, System.Double.format, System.Double.getHashCode);}},{"a":2,"n":"Subproblem2","is":true,"t":8,"pi":[{"n":"p","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":1},{"n":"k1","pt":$n[0].Vector$1(System.Double),"ps":2},{"n":"k2","pt":$n[0].Vector$1(System.Double),"ps":3}],"sn":"Subproblem2","rt":$n[2].Array.type(System.Double),"p":[$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double)]},{"a":2,"n":"Subproblem3","is":true,"t":8,"pi":[{"n":"p","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":1},{"n":"k","pt":$n[0].Vector$1(System.Double),"ps":2},{"n":"d","pt":$n[2].Double,"ps":3}],"sn":"Subproblem3","rt":$n[2].Array.type(System.Double),"p":[$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[2].Double]},{"a":2,"n":"Subproblem4","is":true,"t":8,"pi":[{"n":"p","pt":$n[0].Vector$1(System.Double),"ps":0},{"n":"q","pt":$n[0].Vector$1(System.Double),"ps":1},{"n":"k","pt":$n[0].Vector$1(System.Double),"ps":2},{"n":"d","pt":$n[2].Double,"ps":3}],"sn":"Subproblem4","rt":$n[2].Array.type(System.Double),"p":[$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[0].Vector$1(System.Double),$n[2].Double]},{"a":1,"n":"m_builder","is":true,"t":4,"rt":$n[3].MatrixBuilder$1(System.Double),"sn":"m_builder"},{"a":1,"n":"v_builder","is":true,"t":4,"rt":$n[3].VectorBuilder$1(System.Double),"sn":"v_builder"}]}; }, $n);
    $m("TestGeneralRoboticsToolboxNET.Robot", function () { return {"att":1048577,"a":2,"m":[{"a":2,"n":".ctor","t":1,"sn":"ctor"},{"a":2,"n":".ctor","t":1,"p":[$n[0].Matrix$1(System.Double),$n[0].Matrix$1(System.Double),$n[2].Array.type(System.Int32),$n[2].Array.type(System.Double),$n[2].Array.type(System.Double),$n[2].Array.type(System.Double),$n[2].Array.type(System.Double),System.Array.type(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double)),$n[0].Matrix$1(System.Double),$n[0].Vector$1(System.Double),$n[2].Array.type(System.String),$n[2].String,$n[2].String],"pi":[{"n":"h","pt":$n[0].Matrix$1(System.Double),"ps":0},{"n":"p","pt":$n[0].Matrix$1(System.Double),"ps":1},{"n":"joint_type","pt":$n[2].Array.type(System.Int32),"ps":2},{"n":"joint_lower_limit","dv":null,"o":true,"pt":$n[2].Array.type(System.Double),"ps":3},{"n":"joint_upper_limit","dv":null,"o":true,"pt":$n[2].Array.type(System.Double),"ps":4},{"n":"joint_vel_limit","dv":null,"o":true,"pt":$n[2].Array.type(System.Double),"ps":5},{"n":"joint_acc_limit","dv":null,"o":true,"pt":$n[2].Array.type(System.Double),"ps":6},{"n":"m","dv":null,"o":true,"pt":System.Array.type(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double)),"ps":7},{"n":"r_tool","dv":null,"o":true,"pt":$n[0].Matrix$1(System.Double),"ps":8},{"n":"p_tool","dv":null,"o":true,"pt":$n[0].Vector$1(System.Double),"ps":9},{"n":"joint_names","dv":null,"o":true,"pt":$n[2].Array.type(System.String),"ps":10},{"n":"root_link_name","dv":null,"o":true,"pt":$n[2].String,"ps":11},{"n":"tip_link_name","dv":null,"o":true,"pt":$n[2].String,"ps":12}],"sn":"$ctor1"},{"a":2,"n":"H","t":16,"rt":$n[0].Matrix$1(System.Double),"g":{"a":2,"n":"get_H","t":8,"rt":$n[0].Matrix$1(System.Double),"fg":"H"},"s":{"a":2,"n":"set_H","t":8,"p":[$n[0].Matrix$1(System.Double)],"rt":$n[2].Void,"fs":"H"},"fn":"H"},{"a":2,"n":"Joint_acc_limit","t":16,"rt":$n[2].Array.type(System.Double),"g":{"a":2,"n":"get_Joint_acc_limit","t":8,"rt":$n[2].Array.type(System.Double),"fg":"Joint_acc_limit"},"s":{"a":2,"n":"set_Joint_acc_limit","t":8,"p":[$n[2].Array.type(System.Double)],"rt":$n[2].Void,"fs":"Joint_acc_limit"},"fn":"Joint_acc_limit"},{"a":2,"n":"Joint_lower_limit","t":16,"rt":$n[2].Array.type(System.Double),"g":{"a":2,"n":"get_Joint_lower_limit","t":8,"rt":$n[2].Array.type(System.Double),"fg":"Joint_lower_limit"},"s":{"a":2,"n":"set_Joint_lower_limit","t":8,"p":[$n[2].Array.type(System.Double)],"rt":$n[2].Void,"fs":"Joint_lower_limit"},"fn":"Joint_lower_limit"},{"a":2,"n":"Joint_names","t":16,"rt":$n[2].Array.type(System.String),"g":{"a":2,"n":"get_Joint_names","t":8,"rt":$n[2].Array.type(System.String),"fg":"Joint_names"},"s":{"a":2,"n":"set_Joint_names","t":8,"p":[$n[2].Array.type(System.String)],"rt":$n[2].Void,"fs":"Joint_names"},"fn":"Joint_names"},{"a":2,"n":"Joint_type","t":16,"rt":$n[2].Array.type(System.Int32),"g":{"a":2,"n":"get_Joint_type","t":8,"rt":$n[2].Array.type(System.Int32),"fg":"Joint_type"},"s":{"a":2,"n":"set_Joint_type","t":8,"p":[$n[2].Array.type(System.Int32)],"rt":$n[2].Void,"fs":"Joint_type"},"fn":"Joint_type"},{"a":2,"n":"Joint_upper_limit","t":16,"rt":$n[2].Array.type(System.Double),"g":{"a":2,"n":"get_Joint_upper_limit","t":8,"rt":$n[2].Array.type(System.Double),"fg":"Joint_upper_limit"},"s":{"a":2,"n":"set_Joint_upper_limit","t":8,"p":[$n[2].Array.type(System.Double)],"rt":$n[2].Void,"fs":"Joint_upper_limit"},"fn":"Joint_upper_limit"},{"a":2,"n":"Joint_vel_limit","t":16,"rt":$n[2].Array.type(System.Double),"g":{"a":2,"n":"get_Joint_vel_limit","t":8,"rt":$n[2].Array.type(System.Double),"fg":"Joint_vel_limit"},"s":{"a":2,"n":"set_Joint_vel_limit","t":8,"p":[$n[2].Array.type(System.Double)],"rt":$n[2].Void,"fs":"Joint_vel_limit"},"fn":"Joint_vel_limit"},{"a":2,"n":"M","t":16,"rt":System.Array.type(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double)),"g":{"a":2,"n":"get_M","t":8,"rt":System.Array.type(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double)),"fg":"M"},"s":{"a":2,"n":"set_M","t":8,"p":[System.Array.type(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double))],"rt":$n[2].Void,"fs":"M"},"fn":"M"},{"a":2,"n":"P","t":16,"rt":$n[0].Matrix$1(System.Double),"g":{"a":2,"n":"get_P","t":8,"rt":$n[0].Matrix$1(System.Double),"fg":"P"},"s":{"a":2,"n":"set_P","t":8,"p":[$n[0].Matrix$1(System.Double)],"rt":$n[2].Void,"fs":"P"},"fn":"P"},{"a":2,"n":"P_tool","t":16,"rt":$n[0].Vector$1(System.Double),"g":{"a":2,"n":"get_P_tool","t":8,"rt":$n[0].Vector$1(System.Double),"fg":"P_tool"},"s":{"a":2,"n":"set_P_tool","t":8,"p":[$n[0].Vector$1(System.Double)],"rt":$n[2].Void,"fs":"P_tool"},"fn":"P_tool"},{"a":2,"n":"R_tool","t":16,"rt":$n[0].Matrix$1(System.Double),"g":{"a":2,"n":"get_R_tool","t":8,"rt":$n[0].Matrix$1(System.Double),"fg":"R_tool"},"s":{"a":2,"n":"set_R_tool","t":8,"p":[$n[0].Matrix$1(System.Double)],"rt":$n[2].Void,"fs":"R_tool"},"fn":"R_tool"},{"a":2,"n":"Root_link_name","t":16,"rt":$n[2].String,"g":{"a":2,"n":"get_Root_link_name","t":8,"rt":$n[2].String,"fg":"Root_link_name"},"s":{"a":2,"n":"set_Root_link_name","t":8,"p":[$n[2].String],"rt":$n[2].Void,"fs":"Root_link_name"},"fn":"Root_link_name"},{"a":2,"n":"Tip_link_name","t":16,"rt":$n[2].String,"g":{"a":2,"n":"get_Tip_link_name","t":8,"rt":$n[2].String,"fg":"Tip_link_name"},"s":{"a":2,"n":"set_Tip_link_name","t":8,"p":[$n[2].String],"rt":$n[2].Void,"fs":"Tip_link_name"},"fn":"Tip_link_name"},{"a":1,"backing":true,"n":"<H>k__BackingField","t":4,"rt":$n[0].Matrix$1(System.Double),"sn":"H"},{"a":1,"backing":true,"n":"<Joint_acc_limit>k__BackingField","t":4,"rt":$n[2].Array.type(System.Double),"sn":"Joint_acc_limit"},{"a":1,"backing":true,"n":"<Joint_lower_limit>k__BackingField","t":4,"rt":$n[2].Array.type(System.Double),"sn":"Joint_lower_limit"},{"a":1,"backing":true,"n":"<Joint_names>k__BackingField","t":4,"rt":$n[2].Array.type(System.String),"sn":"Joint_names"},{"a":1,"backing":true,"n":"<Joint_type>k__BackingField","t":4,"rt":$n[2].Array.type(System.Int32),"sn":"Joint_type"},{"a":1,"backing":true,"n":"<Joint_upper_limit>k__BackingField","t":4,"rt":$n[2].Array.type(System.Double),"sn":"Joint_upper_limit"},{"a":1,"backing":true,"n":"<Joint_vel_limit>k__BackingField","t":4,"rt":$n[2].Array.type(System.Double),"sn":"Joint_vel_limit"},{"a":1,"backing":true,"n":"<M>k__BackingField","t":4,"rt":System.Array.type(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double)),"sn":"M"},{"a":1,"backing":true,"n":"<P>k__BackingField","t":4,"rt":$n[0].Matrix$1(System.Double),"sn":"P"},{"a":1,"backing":true,"n":"<P_tool>k__BackingField","t":4,"rt":$n[0].Vector$1(System.Double),"sn":"P_tool"},{"a":1,"backing":true,"n":"<R_tool>k__BackingField","t":4,"rt":$n[0].Matrix$1(System.Double),"sn":"R_tool"},{"a":1,"backing":true,"n":"<Root_link_name>k__BackingField","t":4,"rt":$n[2].String,"sn":"Root_link_name"},{"a":1,"backing":true,"n":"<Tip_link_name>k__BackingField","t":4,"rt":$n[2].String,"sn":"Tip_link_name"}]}; }, $n);
    $m("TestGeneralRoboticsToolboxNET.Transform", function () { return {"att":1048577,"a":2,"m":[{"a":2,"n":".ctor","t":1,"sn":"ctor"},{"a":2,"n":".ctor","t":1,"p":[$n[0].Matrix$1(System.Double),$n[0].Vector$1(System.Double),$n[2].String,$n[2].String],"pi":[{"n":"r","pt":$n[0].Matrix$1(System.Double),"ps":0},{"n":"p","pt":$n[0].Vector$1(System.Double),"ps":1},{"n":"parent_frame_id","dv":null,"o":true,"pt":$n[2].String,"ps":2},{"n":"child_frame_id","dv":null,"o":true,"pt":$n[2].String,"ps":3}],"sn":"$ctor1"},{"a":2,"n":"inv","t":8,"pi":[{"n":"tran1","pt":$n[1].Transform,"ps":0}],"sn":"inv","rt":$n[1].Transform,"p":[$n[1].Transform]},{"a":2,"n":"op_Equality","is":true,"t":8,"pi":[{"n":"tran1","pt":$n[1].Transform,"ps":0},{"n":"tran2","pt":$n[1].Transform,"ps":1}],"sn":"op_Equality","rt":$n[2].Boolean,"p":[$n[1].Transform,$n[1].Transform],"box":function ($v) { return Bridge.box($v, System.Boolean, System.Boolean.toString);}},{"a":2,"n":"op_Inequality","is":true,"t":8,"pi":[{"n":"tran1","pt":$n[1].Transform,"ps":0},{"n":"tran2","pt":$n[1].Transform,"ps":1}],"sn":"op_Inequality","rt":$n[2].Boolean,"p":[$n[1].Transform,$n[1].Transform],"box":function ($v) { return Bridge.box($v, System.Boolean, System.Boolean.toString);}},{"a":2,"n":"op_Multiply","is":true,"t":8,"pi":[{"n":"tran1","pt":$n[1].Transform,"ps":0},{"n":"tran2","pt":$n[1].Transform,"ps":1}],"sn":"op_Multiply","rt":$n[1].Transform,"p":[$n[1].Transform,$n[1].Transform]},{"a":2,"n":"Child_frame_id","t":16,"rt":$n[2].String,"g":{"a":2,"n":"get_Child_frame_id","t":8,"rt":$n[2].String,"fg":"Child_frame_id"},"s":{"a":2,"n":"set_Child_frame_id","t":8,"p":[$n[2].String],"rt":$n[2].Void,"fs":"Child_frame_id"},"fn":"Child_frame_id"},{"a":2,"n":"P","t":16,"rt":$n[0].Vector$1(System.Double),"g":{"a":2,"n":"get_P","t":8,"rt":$n[0].Vector$1(System.Double),"fg":"P"},"s":{"a":2,"n":"set_P","t":8,"p":[$n[0].Vector$1(System.Double)],"rt":$n[2].Void,"fs":"P"},"fn":"P"},{"a":2,"n":"Parent_frame_id","t":16,"rt":$n[2].String,"g":{"a":2,"n":"get_Parent_frame_id","t":8,"rt":$n[2].String,"fg":"Parent_frame_id"},"s":{"a":2,"n":"set_Parent_frame_id","t":8,"p":[$n[2].String],"rt":$n[2].Void,"fs":"Parent_frame_id"},"fn":"Parent_frame_id"},{"a":2,"n":"R","t":16,"rt":$n[0].Matrix$1(System.Double),"g":{"a":2,"n":"get_R","t":8,"rt":$n[0].Matrix$1(System.Double),"fg":"R"},"s":{"a":2,"n":"set_R","t":8,"p":[$n[0].Matrix$1(System.Double)],"rt":$n[2].Void,"fs":"R"},"fn":"R"},{"a":1,"backing":true,"n":"<Child_frame_id>k__BackingField","t":4,"rt":$n[2].String,"sn":"Child_frame_id"},{"a":1,"backing":true,"n":"<P>k__BackingField","t":4,"rt":$n[0].Vector$1(System.Double),"sn":"P"},{"a":1,"backing":true,"n":"<Parent_frame_id>k__BackingField","t":4,"rt":$n[2].String,"sn":"Parent_frame_id"},{"a":1,"backing":true,"n":"<R>k__BackingField","t":4,"rt":$n[0].Matrix$1(System.Double),"sn":"R"}]}; }, $n);
    $m("TestGeneralRoboticsToolboxNET.NormalizeJoints", function () { return {"att":1048577,"a":2,"m":[{"a":2,"n":".ctor","t":1,"sn":"ctor"},{"a":2,"n":".ctor","t":1,"p":[$n[1].Robot,$n[2].Array.type(System.Double)],"pi":[{"n":"robot","pt":$n[1].Robot,"ps":0},{"n":"last_joints","dv":null,"o":true,"pt":$n[2].Array.type(System.Double),"ps":1}],"sn":"$ctor1"},{"a":2,"n":"FindNormalizedJoints","t":8,"pi":[{"n":"joint_index","pt":$n[2].Int32,"ps":0},{"n":"thetas","pt":$n[2].Array.type(System.Double),"ps":1}],"sn":"FindNormalizedJoints","rt":$n[2].Array.type(System.Double),"p":[$n[2].Int32,$n[2].Array.type(System.Double)]},{"a":2,"n":"FindNormalizedJoints","t":8,"pi":[{"n":"joint_index","pt":$n[2].Array.type(System.Int32),"ps":0},{"n":"thetas","pt":$n[2].Array.type(System.Double),"ps":1}],"sn":"FindNormalizedJoints$1","rt":$n[2].Array.type(System.Array.type(System.Double)),"p":[$n[2].Array.type(System.Int32),$n[2].Array.type(System.Double)]},{"a":2,"n":"Normalize","t":8,"pi":[{"n":"joint_index","pt":$n[2].Int32,"ps":0},{"n":"theta","pt":$n[2].Double,"ps":1}],"sn":"Normalize","rt":$n[2].Double,"p":[$n[2].Int32,$n[2].Double],"box":function ($v) { return Bridge.box($v, System.Double, System.Double.format, System.Double.getHashCode);}},{"a":2,"n":"Last_joints","t":16,"rt":$n[2].Array.type(System.Double),"g":{"a":2,"n":"get_Last_joints","t":8,"rt":$n[2].Array.type(System.Double),"fg":"Last_joints"},"s":{"a":2,"n":"set_Last_joints","t":8,"p":[$n[2].Array.type(System.Double)],"rt":$n[2].Void,"fs":"Last_joints"},"fn":"Last_joints"},{"a":2,"n":"Robot","t":16,"rt":$n[1].Robot,"g":{"a":2,"n":"get_Robot","t":8,"rt":$n[1].Robot,"fg":"Robot"},"s":{"a":2,"n":"set_Robot","t":8,"p":[$n[1].Robot],"rt":$n[2].Void,"fs":"Robot"},"fn":"Robot"},{"a":1,"n":"check_limits","t":16,"rt":$n[2].Boolean,"g":{"a":1,"n":"get_check_limits","t":8,"rt":$n[2].Boolean,"fg":"check_limits","box":function ($v) { return Bridge.box($v, System.Boolean, System.Boolean.toString);}},"s":{"a":1,"n":"set_check_limits","t":8,"p":[$n[2].Boolean],"rt":$n[2].Void,"fs":"check_limits"},"fn":"check_limits"},{"a":1,"n":"current_compare","t":16,"rt":$n[2].Double,"g":{"a":1,"n":"get_current_compare","t":8,"rt":$n[2].Double,"fg":"current_compare","box":function ($v) { return Bridge.box($v, System.Double, System.Double.format, System.Double.getHashCode);}},"s":{"a":1,"n":"set_current_compare","t":8,"p":[$n[2].Double],"rt":$n[2].Void,"fs":"current_compare"},"fn":"current_compare"},{"a":1,"n":"current_compare2","t":16,"rt":$n[2].Array.type(System.Double),"g":{"a":1,"n":"get_current_compare2","t":8,"rt":$n[2].Array.type(System.Double),"fg":"current_compare2"},"s":{"a":1,"n":"set_current_compare2","t":8,"p":[$n[2].Array.type(System.Double)],"rt":$n[2].Void,"fs":"current_compare2"},"fn":"current_compare2"},{"a":1,"n":"use_last_joints","t":16,"rt":$n[2].Boolean,"g":{"a":1,"n":"get_use_last_joints","t":8,"rt":$n[2].Boolean,"fg":"use_last_joints","box":function ($v) { return Bridge.box($v, System.Boolean, System.Boolean.toString);}},"s":{"a":1,"n":"set_use_last_joints","t":8,"p":[$n[2].Boolean],"rt":$n[2].Void,"fs":"use_last_joints"},"fn":"use_last_joints"},{"a":1,"n":"m_builder","is":true,"t":4,"rt":$n[3].MatrixBuilder$1(System.Double),"sn":"m_builder"},{"a":1,"n":"v_builder","is":true,"t":4,"rt":$n[3].VectorBuilder$1(System.Double),"sn":"v_builder"},{"a":1,"backing":true,"n":"<Last_joints>k__BackingField","t":4,"rt":$n[2].Array.type(System.Double),"sn":"Last_joints"},{"a":1,"backing":true,"n":"<Robot>k__BackingField","t":4,"rt":$n[1].Robot,"sn":"Robot"},{"a":1,"backing":true,"n":"<check_limits>k__BackingField","t":4,"rt":$n[2].Boolean,"sn":"check_limits","box":function ($v) { return Bridge.box($v, System.Boolean, System.Boolean.toString);}},{"a":1,"backing":true,"n":"<current_compare>k__BackingField","t":4,"rt":$n[2].Double,"sn":"current_compare","box":function ($v) { return Bridge.box($v, System.Double, System.Double.format, System.Double.getHashCode);}},{"a":1,"backing":true,"n":"<current_compare2>k__BackingField","t":4,"rt":$n[2].Array.type(System.Double),"sn":"current_compare2"},{"a":1,"backing":true,"n":"<use_last_joints>k__BackingField","t":4,"rt":$n[2].Boolean,"sn":"use_last_joints","box":function ($v) { return Bridge.box($v, System.Boolean, System.Boolean.toString);}}]}; }, $n);
    $m("TestGeneralRoboticsToolboxNET.InverseKin", function () { return {"att":1048577,"a":2,"m":[{"a":2,"isSynthetic":true,"n":".ctor","t":1,"sn":"ctor"},{"a":2,"n":"robot6_sphericalwrist_invkin","is":true,"t":8,"pi":[{"n":"robot","pt":$n[1].Robot,"ps":0},{"n":"desired_pose","pt":$n[1].Transform,"ps":1},{"n":"last_joints","dv":null,"o":true,"pt":$n[2].Array.type(System.Double),"ps":2}],"sn":"robot6_sphericalwrist_invkin","rt":$n[2].Array.type(System.Array.type(System.Double)),"p":[$n[1].Robot,$n[1].Transform,$n[2].Array.type(System.Double)]},{"a":1,"n":"m_builder","is":true,"t":4,"rt":$n[3].MatrixBuilder$1(System.Double),"sn":"m_builder"},{"a":1,"n":"v_builder","is":true,"t":4,"rt":$n[3].VectorBuilder$1(System.Double),"sn":"v_builder"}]}; }, $n);
    $m("BridgeGeneralRoboticsToolbox.App", function () { return {"att":1048577,"a":2,"m":[{"a":2,"isSynthetic":true,"n":".ctor","t":1,"sn":"ctor"},{"a":2,"n":"Main","is":true,"t":8,"sn":"Main","rt":$n[2].Void}]}; }, $n);
});
