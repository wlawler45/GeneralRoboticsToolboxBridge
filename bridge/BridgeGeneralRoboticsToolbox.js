/**
 * @version 1.0.0.0
 * @copyright Copyright ©  2020
 * @compiler Bridge.NET 17.10.1
 */
Bridge.assembly("BridgeGeneralRoboticsToolbox", function ($asm, globals) {
    "use strict";

    Bridge.define("BridgeGeneralRoboticsToolbox.App", {
        main: function Main () {

            GeneralRoboticsToolboxTests.UnitTest1.RunTests();




        }
    });

    Bridge.define("TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox", {
        statics: {
            fields: {
                v_builder: null,
                m_builder: null
            },
            ctors: {
                init: function () {
                    this.v_builder = MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra.BuilderInstance$1(System.Double).Vector;
                    this.m_builder = MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra.BuilderInstance$1(System.Double).Matrix;
                }
            },
            methods: {
                Hat: function (k) {
                    var khat = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(3, 3);
                    khat.setItem(0, 1, -k.getItem(2));
                    khat.setItem(0, 2, k.getItem(1));
                    khat.setItem(1, 0, k.getItem(2));
                    khat.setItem(1, 2, -k.getItem(0));
                    khat.setItem(2, 0, -k.getItem(1));
                    khat.setItem(2, 1, k.getItem(0));
                    return khat;
                },
                Invhat: function (khat) {
                    var inv = System.Array.init([
                        (-khat.getItem(1, 2) + khat.getItem(2, 1)), 
                        (khat.getItem(0, 2) - khat.getItem(2, 0)), 
                        (-khat.getItem(0, 1) + khat.getItem(1, 0))
                    ], System.Double);
                    var output = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(inv);
                    output = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(output, 2);
                    return output;
                },
                Rot: function (k, theta) {
                    var I = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);
                    var khat = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(k);
                    var khat2 = khat.Multiply$1(khat);
                    return (MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Addition$2(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Addition$2(I, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply(Math.sin(theta), khat)), MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply((1.0 - Math.cos(theta)), khat2)));
                },
                R2rot: function (R) {
                    var R1 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Subtraction$2(R, R.Transpose());
                    var sin_theta = R1.L2Norm() / 2.0;
                    var cos_theta = (R.Trace() - 1.0) / 2.0;
                    var theta = Math.atan2(sin_theta, cos_theta);
                    var k;
                    if (sin_theta < (1E-06)) {
                        if (cos_theta > 0) {
                            k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 1.0], System.Double));

                            return { Item1: k, Item2: 0 };
                        } else {
                            var eye = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);
                            var B = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply((0.5), (MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Addition$2(R, eye)));
                            k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([Math.sqrt(B.getItem(0, 0)), Math.sqrt(B.getItem(1, 1)), Math.sqrt(B.getItem(2, 2))], System.Double));
                            if (Math.abs(k.getItem(0)) > 1E-06) {
                                k.setItem(1, k.getItem(1) * Bridge.Int.sign(B.getItem(0, 1) / k.getItem(0)));
                                k.setItem(2, k.getItem(2) * Bridge.Int.sign(B.getItem(0, 2) / k.getItem(0)));
                            } else if (Math.abs(k.getItem(1)) > 1E-06) {
                                k.setItem(2, k.getItem(2) * Bridge.Int.sign(B.getItem(0, 2) / k.getItem(1)));
                            }
                            return { Item1: k, Item2: Math.PI };
                        }
                    }
                    k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 0.0], System.Double));
                    var inv = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Invhat(R1);
                    for (var i = 0; i < k.Count; i = (i + 1) | 0) {
                        k.setItem(i, inv.getItem(i) / (2.0 * sin_theta));
                    }
                    return { Item1: k, Item2: theta };

                },
                Screw_matrix: function (r) {
                    var I = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(6);
                    var hat = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(r);
                    for (var i = 0; i < 3; i = (i + 1) | 0) {
                        for (var j = 3; j < 6; j = (j + 1) | 0) {
                            I.setItem(i, j, hat.getItem(i, ((j - 3) | 0)));
                        }
                    }
                    return I;
                },
                Q2R: function (q) {
                    var I = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);

                    var hat = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(q.SubVector(1, 3));

                    var qhat2 = hat.Multiply$1(hat);
                    return MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Addition$2(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Addition$2(I, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply(2 * q.getItem(0), hat)), MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply(2, qhat2));

                },
                R2Q: function (R) {
                    var tr = R.Trace();
                    var q;
                    if (tr > 0) {
                        var S = 2 * Math.sqrt(tr + 1);

                        q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([(0.25 * S), ((R.getItem(2, 1) - R.getItem(1, 2)) / S), ((R.getItem(0, 2) - R.getItem(2, 0)) / S), ((R.getItem(1, 0) - R.getItem(0, 1)) / S)], System.Double));

                    } else if (R.getItem(0, 0) > R.getItem(1, 1) && R.getItem(0, 0) > R.getItem(2, 2)) {
                        var S1 = 2 * Math.sqrt(1 + R.getItem(0, 0) - R.getItem(1, 1) - R.getItem(2, 2));
                        q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([((R.getItem(2, 1) - R.getItem(1, 2)) / S1), (0.25 * S1), ((R.getItem(0, 1) + R.getItem(1, 0)) / S1), ((R.getItem(0, 2) + R.getItem(2, 0)) / S1)], System.Double));
                    } else if (R.getItem(1, 1) > R.getItem(2, 2)) {
                        var S2 = 2 * Math.sqrt(1 - R.getItem(0, 0) + R.getItem(1, 1) - R.getItem(2, 2));
                        q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([((R.getItem(0, 2) - R.getItem(2, 0)) / S2), ((R.getItem(0, 1) + R.getItem(1, 0)) / S2), (0.25 * S2), ((R.getItem(1, 2) + R.getItem(2, 1)) / S2)], System.Double));

                    } else {
                        var S3 = 2 * Math.sqrt(1 - R.getItem(0, 0) - R.getItem(1, 1) + R.getItem(2, 2));
                        q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([((R.getItem(1, 0) - R.getItem(0, 1)) / S3), ((R.getItem(0, 2) + R.getItem(2, 0)) / S3), ((R.getItem(1, 2) + R.getItem(2, 1)) / S3), (0.25 * S3)], System.Double));
                    }
                    return q;

                },
                Q2Rot: function (q) {
                    var theta = 2 * Math.acos(q.getItem(0));
                    var k;
                    if (Math.abs(theta) < (Math.pow(10, -6))) {
                        k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 1.0], System.Double));

                        return { Item1: k, Item2: 0 };
                    }
                    k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 1.0], System.Double));
                    for (var i = 1; i < ((k.Count + 1) | 0); i = (i + 1) | 0) {
                        k.setItem(((i - 1) | 0), q.getItem(i) / Math.sin(theta / 2.0));
                    }
                    return { Item1: k, Item2: theta };
                },
                Rot2Q: function (k, theta) {
                    var output = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([Math.cos(theta / 2.0), k[System.Array.index(0, k)] * Math.sin(theta / 2.0), k[System.Array.index(1, k)] * Math.sin(theta / 2.0), k[System.Array.index(2, k)] * Math.sin(theta / 2.0)], System.Double));
                    return output;
                },
                Quatcomplement: function (q) {
                    var output = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([q.getItem(0), -1 * q.getItem(1), -1 * q.getItem(2), -1 * q.getItem(3)], System.Double));
                    return output;
                },
                Quatproduct: function (q) {
                    var I = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);
                    var Q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(4, 4);
                    var hats = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(q.SubVector(1, 3));
                    Q.setItem(0, 0, q.getItem(0));

                    for (var i = 1; i < 4; i = (i + 1) | 0) {
                        Q.setItem(0, i, -q.getItem(i));
                    }
                    for (var i1 = 1; i1 < 4; i1 = (i1 + 1) | 0) {
                        Q.setItem(i1, 0, q.getItem(i1));
                    }
                    for (var i2 = 1; i2 < 4; i2 = (i2 + 1) | 0) {
                        for (var j = 1; j < 4; j = (j + 1) | 0) {
                            Q.setItem(i2, j, q.getItem(0) * I.getItem(((i2 - 1) | 0), ((j - 1) | 0)) + hats.getItem(((i2 - 1) | 0), ((j - 1) | 0)));
                        }

                    }
                    return Q;
                },
                Quatjacobian: function (q) {
                    var I = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);
                    var J = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(4, 3);
                    var hats = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(q.SubVector(1, 3));
                    for (var i = 0; i < 3; i = (i + 1) | 0) {
                        J.setItem(0, i, 0.5 * -q.getItem(((i + 1) | 0)));
                    }
                    for (var i1 = 1; i1 < 4; i1 = (i1 + 1) | 0) {
                        for (var j = 0; j < 3; j = (j + 1) | 0) {
                            J.setItem(i1, j, 0.5 * (q.getItem(0) * I.getItem(((i1 - 1) | 0), j) - hats.getItem(((i1 - 1) | 0), j)));
                        }

                    }
                    return J;
                },
                Rpy2R: function (rpy) {
                    var k;
                    var rotation1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot((k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 1.0], System.Double))), rpy.getItem(2));
                    var rotation2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot((k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([0.0, 1.0, 0.0], System.Double))), rpy.getItem(1));
                    var rotation3 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot((k = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([1.0, 0.0, 0.0], System.Double))), rpy.getItem(0));
                    return rotation1.Multiply$1(rotation2).Multiply$1(rotation3);
                },
                R2Rpy: function (R) {
                    var output;
                    var r = Math.atan2(R.getItem(2, 1), R.getItem(2, 2));
                    var y = Math.atan2(R.getItem(1, 0), R.getItem(0, 0));

                    var normie = (R.SubMatrix(2, 1, 1, 2)).L2Norm();
                    var p = Math.atan2(-R.getItem(2, 0), normie);
                    return (output = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([r, p, y], System.Double)));
                },
                Fwdkin: function (robot, theta) {
                    var $t, $t1, $t2, $t3, $t4, $t5;

                    if (robot.Joint_lower_limit != null && robot.Joint_upper_limit != null) {
                        for (var i = 0; i < theta.length; i = (i + 1) | 0) {
                            if (!(($t = robot.Joint_lower_limit)[System.Array.index(i, $t)] < theta[System.Array.index(i, theta)] && theta[System.Array.index(i, theta)] < ($t1 = robot.Joint_upper_limit)[System.Array.index(i, $t1)])) {
                                throw new System.ArgumentException.$ctor1(System.String.format("Joint angle for joint {0} out of range", [Bridge.box(i, System.Int32)]));
                            }
                        }
                    }
                    var p;
                    p = robot.P.Column(0);
                    var R = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);
                    for (var i1 = 0; i1 < robot.Joint_type.length; i1 = (i1 + 1) | 0) {
                        if (($t2 = robot.Joint_type)[System.Array.index(i1, $t2)] === 0 || ($t3 = robot.Joint_type)[System.Array.index(i1, $t3)] === 2) {
                            R = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(robot.H.Column(i1), theta[System.Array.index(i1, theta)]));
                        } else if (($t4 = robot.Joint_type)[System.Array.index(i1, $t4)] === 1 || ($t5 = robot.Joint_type)[System.Array.index(i1, $t5)] === 3) {
                            p = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply(theta[System.Array.index(i1, theta)], R), robot.H.Column(i1)));
                        }
                        p = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R, robot.P.Column(((i1 + 1) | 0))));
                    }

                    if (robot.R_tool != null && robot.P_tool != null) {

                        p = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p, R.Multiply$2(robot.P_tool));
                        R = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R, robot.R_tool);


                    }
                    var output = new TestGeneralRoboticsToolboxNET.Transform.$ctor1(R, p);

                    return output;

                },
                Robotjacobian: function (robot, theta) {
                    var $t, $t1, $t2, $t3, $t4, $t5, $t6, $t7, $t8;

                    if (robot.Joint_lower_limit != null && robot.Joint_upper_limit != null) {
                        for (var k = 0; k < theta.length; k = (k + 1) | 0) {
                            if (!(($t = robot.Joint_lower_limit)[System.Array.index(k, $t)] < theta[System.Array.index(k, theta)] && theta[System.Array.index(k, theta)] < ($t1 = robot.Joint_upper_limit)[System.Array.index(k, $t1)])) {
                                throw new System.ArgumentException.$ctor1(System.String.format("Joint angles out of range", null));
                            }
                        }
                    }

                    var hi = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(robot.H.RowCount, robot.H.ColumnCount);
                    var p0i = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(robot.P.RowCount, robot.P.ColumnCount);
                    var p;
                    p = robot.P.Column(0);
                    var R = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseIdentity(3);

                    p0i.SetColumn$1(0, p);



                    for (var k1 = 0; k1 < robot.Joint_type.length; k1 = (k1 + 1) | 0) {
                        if (($t2 = robot.Joint_type)[System.Array.index(k1, $t2)] === 0 || ($t3 = robot.Joint_type)[System.Array.index(k1, $t3)] === 2) {
                            R = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(robot.H.Column(k1), theta[System.Array.index(k1, theta)]));
                        } else if (($t4 = robot.Joint_type)[System.Array.index(k1, $t4)] === 1 || ($t5 = robot.Joint_type)[System.Array.index(k1, $t5)] === 3) {
                            p = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply(theta[System.Array.index(k1, theta)], R), robot.H.Column(k1)));
                        }
                        p = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R, robot.P.Column(((k1 + 1) | 0))));
                        p0i.SetColumn$1(((k1 + 1) | 0), p);
                        hi.SetColumn$1(k1, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R, robot.H.Column(k1)));

                    }
                    var p0T = p0i.Column(robot.Joint_type.length);


                    if (robot.P_tool != null) {
                        p0T = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p0T, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R, robot.P_tool));
                    }
                    var J = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(6, robot.Joint_type.length);
                    var i = 0;
                    var j = 0;
                    while (i < robot.Joint_type.length) {
                        if (($t6 = robot.Joint_type)[System.Array.index(i, $t6)] === 0) {
                            J.SetColumn$2(j, 0, 3, hi.Column(i));
                            J.SetColumn$2(j, 3, 3, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(hi.Column(i)), (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(p0T, p0i.Column(i)))));
                        } else if (($t7 = robot.Joint_type)[System.Array.index(i, $t7)] === 1) {
                            J.SetColumn$2(j, 3, 3, hi.Column(i));
                        } else if (($t8 = robot.Joint_type)[System.Array.index(i, $t8)] === 3) {
                            J.SetColumn$2(j, 3, 3, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(hi.Column(((i + 2) | 0)), theta[System.Array.index(((i + 2) | 0), theta)]), hi.Column(i)));
                            J.SetColumn$2(((j + 1) | 0), 0, 3, hi.Column(((i + 2) | 0)));
                            J.SetColumn$2(((j + 1) | 0), 3, 3, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(hi.Column(((i + 2) | 0))), (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(p0T, p0i.Column(((i + 2) | 0))))));
                            J = J.SubMatrix(0, J.RowCount, 0, ((J.ColumnCount - 1) | 0));
                            i = (i + 2) | 0;
                            j = (j + 1) | 0;
                        }
                        i = (i + 1) | 0;
                        j = (j + 1) | 0;
                    }
                    return J;




                },
                Cross: function (left, right) {
                    if ((left.Count !== 3 || right.Count !== 3)) {
                        var message = "Vectors must have a length of 3.";
                        throw new System.Exception(message);
                    }
                    var result = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.Dense$2(3);
                    result.setItem(0, left.getItem(1) * right.getItem(2) - left.getItem(2) * right.getItem(1));
                    result.setItem(1, -left.getItem(0) * right.getItem(2) + left.getItem(2) * right.getItem(0));
                    result.setItem(2, left.getItem(0) * right.getItem(1) - left.getItem(1) * right.getItem(0));

                    return result;
                },
                Subproblem0: function (p, q, k) {
                    var min = 4.94065645841247E-324;
                    if (!(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(k, p) < min && MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(k, q) < min)) {
                        throw new System.ArgumentException.$ctor1(System.String.format("k must be perpendicular to p and q", null));
                    }
                    var ep = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(p, p.L2Norm());
                    var eq = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(q, q.L2Norm());
                    var theta = 2 * Math.atan2((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(ep, eq)).L2Norm(), (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(ep, eq)).L2Norm());
                    if (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(k, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Cross(p, q)) < 0) {
                        return -theta;
                    }
                    return theta;
                },
                Subproblem1: function (p, q, k) {
                    var min = 4.94065645841247E-324;
                    var minus = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(p, q);
                    if (minus.L2Norm() < Math.sqrt(min)) {
                        return 0.0;
                    }
                    k = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(k, k.L2Norm());
                    var pp = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(p, (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(p, k), k)));
                    var qp = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(q, (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(q, k), k)));

                    var epp = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(pp, pp.L2Norm());
                    var eqp = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(qp, qp.L2Norm());

                    var theta = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem0(epp, eqp, k);
                    if (Math.abs(p.L2Norm() - q.L2Norm()) > (p.L2Norm() * (Math.pow(10, -8)))) {
                        System.Console.WriteLine("WARNING:||p|| and ||q|| must be the same!!!");
                    }

                    return theta;
                },
                Subproblem2: function (p, q, k1, k2) {
                    var min = 4.94065645841247E-324;
                    var k12 = k1.DotProduct(k2);
                    var pk = p.DotProduct(k2);
                    var qk = q.DotProduct(k1);
                    if (Math.abs(1 - Math.pow(k12, 2)) < min) {
                        System.Console.WriteLine("WARNING:No solution found k1 != k2");
                        return System.Array.init(0, 0, System.Double);
                    }
                    var amatrix = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.Dense$1(2, 2);
                    amatrix.setItem(0, 0, k12);
                    amatrix.setItem(0, 1, -1);
                    amatrix.setItem(1, 0, -1);
                    amatrix.setItem(1, 1, k12);
                    var avector = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.DenseOfArray(System.Array.init([pk, qk], System.Double));
                    var a = amatrix.Multiply$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(avector, (Math.pow(k12, 2) - 1)));
                    var bb = p.DotProduct(p) - a.DotProduct(a) - 2 * a.getItem(0) * a.getItem(1) * k12;
                    if (Math.abs(bb) < min) {
                        bb = 0;
                    }
                    if (bb < 0) {
                        System.Console.WriteLine("WARNING:No solution found no intersection found between cones");
                        return System.Array.init(0, 0, System.Double);
                    }
                    var gamma = Math.sqrt(bb) / TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Cross(k1, k2).L2Norm();
                    if (Math.abs(gamma) < min) {
                        var cmalt = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseOfRowVectors([k1, k2, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Cross(k1, k2)]);
                        cmalt = cmalt.Transpose();
                        var c1vecalt = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.Dense$2(3);
                        c1vecalt.setItem(0, a.getItem(0));
                        c1vecalt.setItem(1, a.getItem(1));
                        c1vecalt.setItem(2, gamma);

                        var c1alt = cmalt.Multiply$2(c1vecalt);
                        var theta2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(k2, p, c1alt);
                        var theta1 = -TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(k1, q, c1alt);
                        var thetasfirst = System.Array.init([theta1, theta2], System.Double);


                        return thetasfirst;
                    }
                    var cm = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.m_builder.DenseOfRowVectors([k1, k2, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Cross(k1, k2)]);
                    cm = cm.Transpose();
                    var c1vec = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.Dense$2(3);
                    c1vec.setItem(0, a.getItem(0));
                    c1vec.setItem(1, a.getItem(1));
                    c1vec.setItem(2, gamma);
                    var c2vec = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.Dense$2(3);
                    c2vec.setItem(0, a.getItem(0));
                    c2vec.setItem(1, a.getItem(1));
                    c2vec.setItem(2, -gamma);
                    var c1 = cm.Multiply$2(c1vec);
                    var c2 = cm.Multiply$2(c2vec);

                    var theta1_1 = -TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(q, c1, k1);
                    var theta1_2 = -TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(q, c2, k1);
                    var theta2_1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(p, c1, k2);
                    var theta2_2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(p, c2, k2);
                    var thetas = System.Array.init([theta1_1, theta2_1, theta1_2, theta2_2], System.Double);
                    return thetas;
                },
                Subproblem3: function (p, q, k, d) {

                    var pp = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(p, (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(p, k)), k)));
                    var qp = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(q, (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(q, k)), k)));
                    var dpsq = Math.pow(d, 2) - Math.pow((k.DotProduct(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(p, q))), 2);
                    var bb = -(pp.DotProduct(pp) + qp.DotProduct(qp) - dpsq) / (2 * pp.L2Norm() * qp.L2Norm());
                    if (dpsq < 0 || Math.abs(bb) > 1) {
                        System.Console.WriteLine("No solution no rotation can achieve specified distance");
                        return System.Array.init(0, 0, System.Double);
                    }
                    var theta = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(pp, pp.L2Norm()), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(qp, qp.L2Norm()), k);
                    var phi = Math.acos(bb);
                    if (Math.abs(phi) > 0) {
                        return System.Array.init([theta + phi, theta - phi], System.Double);
                    } else {
                        return System.Array.init([theta], System.Double);
                    }

                },
                Subproblem4: function (p, q, k, d) {

                    var hatted = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(k);
                    var a = hatted.LeftMultiply(p).DotProduct(q);

                    var b = -1 * hatted.LeftMultiply(hatted.LeftMultiply(q)).DotProduct(p);
                    var c = d - (p.DotProduct(q) - b);
                    var phi = Math.atan2(b, a);
                    var dprep = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.v_builder.Dense$2(2);
                    dprep.setItem(0, a);
                    dprep.setItem(1, b);
                    d = c / dprep.L2Norm();
                    if (d > 1) {
                        return System.Array.init(0, 0, System.Double);
                    }
                    var psi = Math.asin(d);

                    return System.Array.init([-phi + psi, -phi - psi + Math.PI], System.Double);

                }
            }
        },
        ctors: {
            ctor: function () {
                this.$initialize();
            }
        }
    });

    Bridge.define("TestGeneralRoboticsToolboxNET.InverseKin", {
        statics: {
            fields: {
                v_builder: null,
                m_builder: null
            },
            ctors: {
                init: function () {
                    this.v_builder = MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra.BuilderInstance$1(System.Double).Vector;
                    this.m_builder = MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra.BuilderInstance$1(System.Double).Matrix;
                }
            },
            methods: {
                robot6_sphericalwrist_invkin: function (robot, desired_pose, last_joints) {
                    var $t, $t1, $t2, $t3, $t4, $t5, $t6;
                    if (last_joints === void 0) { last_joints = null; }

                    if (!Bridge.referenceEquals(robot.R_tool, null) && !Bridge.referenceEquals(robot.P_tool, null)) {

                        var transposed = robot.R_tool.Transpose();
                        desired_pose.R = desired_pose.R.Multiply$1(transposed);
                        desired_pose.P = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(desired_pose.P, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(desired_pose.R, robot.P_tool));
                    }
                    var H = robot.H.Clone();
                    var P = robot.P.Clone();
                    var theta_v = new (System.Collections.Generic.List$1(System.Array.type(System.Double))).ctor();
                    var zeros = TestGeneralRoboticsToolboxNET.InverseKin.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 0.0], System.Double));
                    var ex = TestGeneralRoboticsToolboxNET.InverseKin.v_builder.DenseOfArray(System.Array.init([1.0, 0.0, 0.0], System.Double));
                    var ey = TestGeneralRoboticsToolboxNET.InverseKin.v_builder.DenseOfArray(System.Array.init([0.0, 1.0, 0.0], System.Double));
                    var ez = TestGeneralRoboticsToolboxNET.InverseKin.v_builder.DenseOfArray(System.Array.init([0.0, 0.0, 1.0], System.Double));
                    if (!P.Column(4).equalsT(zeros)) {
                        var P4_d = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(P.Column(4), H.Column(3));
                        if (!(P.Column(4).Subtract$1(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(P4_d, H.Column(3))).equalsT(zeros))) {
                            throw new System.ArgumentException.$ctor1(System.String.format("Robot may not have spherical wrist", null));
                        }
                        P.SetColumn$1(3, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(P.Column(3), P.Column(4)));
                        P.SetColumn$1(4, zeros);
                    }
                    if (!P.Column(5).equalsT(zeros)) {
                        var P5_d = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(P.Column(5), H.Column(5));
                        if (!(P.Column(5).Subtract$1(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(P5_d, H.Column(5))).equalsT(zeros))) {
                            throw new System.ArgumentException.$ctor1(System.String.format("Robot may not have spherical wrist", null));
                        }
                        P.SetColumn$1(6, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(P.Column(6), P.Column(5)));
                        P.SetColumn$1(5, zeros);
                    }

                    var d1 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(ey, (P.Column(1).Add$1(P.Column(2)).Add$1(P.Column(3))));
                    var v1 = desired_pose.P.Subtract$1(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(desired_pose.R, P.Column(6)));
                    var Q1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem4(ey, v1, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_UnaryNegation(H.Column(0)), d1);
                    var normalize = new TestGeneralRoboticsToolboxNET.NormalizeJoints.$ctor1(robot, last_joints);

                    var first_normalize = normalize.FindNormalizedJoints(0, Q1);
                    $t = Bridge.getEnumerator(first_normalize);
                    try {
                        while ($t.moveNext()) {
                            var q1 = $t.Current;
                            var R01 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(H.Column(0), q1);
                            var p26_q1 = R01.TransposeThisAndMultiply$1(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(desired_pose.P, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(desired_pose.R, P.Column(6)))).Subtract$1(P.Column(0).Add$1(P.Column(1)));
                            var d3 = p26_q1.L2Norm();
                            var v3 = P.Column(2);
                            var p3 = P.Column(3);
                            var Q3 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem3(p3, v3, H.Column(2), d3);
                            var second_normalize = normalize.FindNormalizedJoints(2, Q3);
                            $t1 = Bridge.getEnumerator(second_normalize);
                            try {
                                while ($t1.moveNext()) {
                                    var q3 = $t1.Current;
                                    var R23 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(H.Column(2), q3);
                                    var v2 = p26_q1;
                                    var p2 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(P.Column(2), MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R23, P.Column(3)));
                                    var q2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(p2, v2, H.Column(1));
                                    var q2_array = normalize.FindNormalizedJoints(1, System.Array.init([q2], System.Double));
                                    if (q2_array.length === 0) {
                                        continue;
                                    }
                                    q2 = q2_array[System.Array.index(0, q2_array)];
                                    var R12 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(H.Column(1), q2);
                                    var R03 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R01, R12), R23);
                                    var R36 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R03.Transpose(), desired_pose.R);
                                    var v4 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R36, H.Column(5));
                                    var p4 = H.Column(5);
                                    var Q4_Q5 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem2(p4, v4, H.Column(3), H.Column(4));
                                    var third_normalize = normalize.FindNormalizedJoints$1(System.Array.init([3, 4], System.Int32), Q4_Q5);
                                    System.Console.WriteLine(System.String.format("size of qs {0}, {1}", Bridge.box(third_normalize[System.Array.index(0, third_normalize)].length, System.Int32), Bridge.box(third_normalize[System.Array.index(1, third_normalize)].length, System.Int32)));
                                    var minoftwo = Math.min(third_normalize[System.Array.index(0, third_normalize)].length, third_normalize[System.Array.index(1, third_normalize)].length);
                                    System.Console.WriteLine(minoftwo);
                                    for (var q = 0; q < minoftwo; q = (q + 1) | 0) {
                                        var R35 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(H.Column(3), ($t2 = third_normalize[System.Array.index(0, third_normalize)])[System.Array.index(q, $t2)]), TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(H.Column(4), ($t3 = third_normalize[System.Array.index(1, third_normalize)])[System.Array.index(q, $t3)]));
                                        var R05 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R03, R35);
                                        var R56 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(R05.Transpose(), desired_pose.R);
                                        var v6 = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(R56, H.Column(4));
                                        var p6 = H.Column(4);
                                        var q6 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(p6, v6, H.Column(5));
                                        var q6_array = normalize.FindNormalizedJoints(5, System.Array.init([q6], System.Double));
                                        if (q6_array.length === 0) {
                                            continue;
                                        }
                                        q6 = q6_array[System.Array.index(0, q6_array)];

                                        var theta_v_entry = System.Array.init([q1, q2, q3, ($t4 = third_normalize[System.Array.index(0, third_normalize)])[System.Array.index(q, $t4)], ($t5 = third_normalize[System.Array.index(1, third_normalize)])[System.Array.index(q, $t5)], q6], System.Double);
                                        theta_v.add(theta_v_entry);


                                    }


                                }
                            } finally {
                                if (Bridge.is($t1, System.IDisposable)) {
                                    $t1.System$IDisposable$Dispose();
                                }
                            }
                        }
                    } finally {
                        if (Bridge.is($t, System.IDisposable)) {
                            $t.System$IDisposable$Dispose();
                        }
                    }

                    if (!Bridge.referenceEquals(last_joints, null)) {
                        var theta_dist_arr = System.Array.init(theta_v.Count, 0, System.Double);


                        for (var i = 0; i < theta_v.Count; i = (i + 1) | 0) {

                            var theta_v_vec = TestGeneralRoboticsToolboxNET.InverseKin.v_builder.DenseOfArray(theta_v.getItem(i));
                            var theta_dist_vec = theta_v_vec.Subtract(last_joints[System.Array.index(i, last_joints)]);
                            theta_dist_arr[System.Array.index(i, theta_dist_arr)] = theta_dist_vec.L2Norm();
                        }
                        var index = 0;
                        var theta_dist_list = ($t6 = System.Double, System.Linq.Enumerable.from(theta_dist_arr, $t6).toList($t6));
                        var returned = new (System.Collections.Generic.List$1(System.Array.type(System.Double))).ctor();
                        for (var z = 0; z < theta_dist_list.Count; z = (z + 1) | 0) {
                            index = theta_dist_list.indexOf(System.Linq.Enumerable.from(theta_dist_list, System.Double).min());
                            returned.setItem(z, theta_v.getItem(index));
                            theta_dist_list.removeAt(index);
                        }
                        return returned.ToArray();
                    } else {
                        return theta_v.ToArray();
                    }
                }
            }
        }
    });

    Bridge.define("TestGeneralRoboticsToolboxNET.NormalizeJoints", {
        statics: {
            fields: {
                v_builder: null,
                m_builder: null
            },
            ctors: {
                init: function () {
                    this.v_builder = MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra.BuilderInstance$1(System.Double).Vector;
                    this.m_builder = MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra.BuilderInstance$1(System.Double).Matrix;
                }
            }
        },
        fields: {
            Robot: null,
            Last_joints: null,
            check_limits: false,
            use_last_joints: false,
            current_compare: 0,
            current_compare2: null
        },
        ctors: {
            ctor: function () {
                this.$initialize();
            },
            $ctor1: function (robot, last_joints) {
                if (last_joints === void 0) { last_joints = null; }

                this.$initialize();
                this.Robot = robot;
                this.Last_joints = last_joints;
                this.check_limits = this.Robot.Joint_upper_limit != null && this.Robot.Joint_lower_limit != null;
                this.use_last_joints = !Bridge.referenceEquals(last_joints, null);
            }
        },
        methods: {
            Normalize: function (joint_index, theta) {
                var $t, $t1, $t2;
                var u = ($t = this.Robot.Joint_upper_limit)[System.Array.index(joint_index, $t)];
                var l = ($t1 = this.Robot.Joint_lower_limit)[System.Array.index(joint_index, $t1)];
                if (this.check_limits) {

                    if (!(l < theta && theta < u)) {
                        var a = TestGeneralRoboticsToolboxNET.NormalizeJoints.v_builder.DenseOfArray(System.Array.init([-1.0, 1.0], System.Double));
                        a = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$2(a, 2), Math.PI);
                        var b = a.Add(theta);
                        var index = -1;
                        for (var i = 0; i < b.Count; i = (i + 1) | 0) {
                            if (l < b.getItem(i) && b.getItem(i) < u) {
                                index = i;
                                break;
                            }
                        }
                        if (index === -1) {
                            return Bridge.getDefaultValue(System.Double);
                        } else {
                            theta = theta + a.getItem(index);
                        }
                    }
                }
                if (this.use_last_joints) {
                    var diff = ($t2 = this.Last_joints)[System.Array.index(joint_index, $t2)] - theta;

                    var n_diff = Bridge.Int.clip32(Math.floor(diff / (6.2831853071795862)));
                    var r_diff = diff % (6.2831853071795862);
                    if (r_diff > Math.PI) {
                        n_diff = (n_diff + 1) | 0;
                    }
                    if (Math.abs(n_diff) > 0) {
                        if (!this.check_limits) {
                            theta += 6.2831853071795862 * n_diff;
                        } else {
                            var theta_vs = theta + 6.2831853071795862;

                            var theta_v = TestGeneralRoboticsToolboxNET.NormalizeJoints.v_builder.Dense$2(Math.abs(n_diff));
                            for (var i1 = 0; i1 < theta_v.Count; i1 = (i1 + 1) | 0) {
                                if (n_diff > 0) {
                                    theta_v.setItem(i1, (n_diff - i1) | 0);
                                } else if (n_diff < 0) {
                                    theta_v.setItem(i1, (n_diff + i1) | 0);
                                }
                            }
                            theta_v = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$2(theta_v, 2), Math.PI);
                            theta_v = theta_v.Add(theta);
                            var index1 = -1;
                            for (var i2 = 0; i2 < theta_v.Count; i2 = (i2 + 1) | 0) {
                                if (l < theta_v.getItem(i2) && theta_v.getItem(i2) < u) {
                                    index1 = i2;
                                    break;
                                }
                            }
                            if (index1 !== -1) {
                                theta = theta_v.getItem(index1);
                            }
                        }
                    }


                }
                return theta;
            },
            FindNormalizedJoints: function (joint_index, thetas) {
                var $t, $t1, $t2, $t3;
                var theta_normed = new (System.Collections.Generic.List$1(System.Double)).ctor();
                $t = Bridge.getEnumerator(thetas);
                try {
                    while ($t.moveNext()) {
                        var t1 = $t.Current;
                        var t3 = this.Normalize(joint_index, t1);
                        if (t3 !== Bridge.getDefaultValue(System.Double)) {
                            theta_normed.add(t3);
                        }
                    }
                } finally {
                    if (Bridge.is($t, System.IDisposable)) {
                        $t.System$IDisposable$Dispose();
                    }
                }

                if (!this.use_last_joints || theta_normed.Count < 2) {
                    var output = System.Array.init(theta_normed.Count, 0, System.Double);
                    for (var i = 0; i < theta_normed.Count; i = (i + 1) | 0) {
                        output[System.Array.index(i, output)] = theta_normed.getItem(i);
                    }
                    return output;
                }
                var theta_normed_vec = TestGeneralRoboticsToolboxNET.NormalizeJoints.v_builder.DenseOfArray(theta_normed.ToArray());

                var theta_last = ($t1 = this.Last_joints)[System.Array.index(joint_index, $t1)];

                var theta_dist_vec = theta_normed_vec.Subtract(theta_last);
                theta_dist_vec = theta_dist_vec.PointwiseAbs();
                var theta_dist = theta_dist_vec.ToArray();
                var theta_ret1 = new (System.Collections.Generic.List$1(System.Double)).ctor();
                $t2 = Bridge.getEnumerator(theta_normed);
                try {
                    while ($t2.moveNext()) {
                        var theta = $t2.Current;
                        this.current_compare = theta;
                        if (Math.abs(this.current_compare - theta_last) < 1.5707963267948966) {
                            theta_ret1.add(theta);
                        }
                    }
                } finally {
                    if (Bridge.is($t2, System.IDisposable)) {
                        $t2.System$IDisposable$Dispose();
                    }
                }
                if (theta_ret1.Count === 1) {
                    return theta_ret1.ToArray();
                }
                if (theta_ret1.Count === 0) {
                    theta_ret1 = theta_normed;
                }
                var returned = System.Array.init(theta_dist.length, 0, System.Double);
                var index = 0;
                var theta_dist_list = ($t3 = System.Double, System.Linq.Enumerable.from(theta_dist, $t3).toList($t3));
                for (var z = 0; z < theta_dist.length; z = (z + 1) | 0) {
                    index = theta_dist_list.indexOf(System.Linq.Enumerable.from(theta_dist_list, System.Double).min());
                    returned[System.Array.index(z, returned)] = theta_normed.getItem(index);
                    theta_dist_list.removeAt(index);
                }
                return returned;


            },
            FindNormalizedJoints$1: function (joint_index, thetas) {
                var $t, $t1, $t2, $t3;
                var theta_normed = new (System.Collections.Generic.List$1(System.Collections.Generic.List$1(System.Double))).ctor();

                $t = Bridge.getEnumerator(thetas);
                try {
                    while ($t.moveNext()) {
                        var t1 = $t.Current;
                        var theta_normed_intermed = new (System.Collections.Generic.List$1(System.Double)).ctor();
                        for (var i = 0; i < joint_index.length; i = (i + 1) | 0) {
                            var t4 = this.Normalize(joint_index[System.Array.index(i, joint_index)], t1);
                            if (t4 !== Bridge.getDefaultValue(System.Double)) {
                                theta_normed_intermed.add(t4);
                            }
                        }
                        theta_normed.add(theta_normed_intermed);
                    }
                } finally {
                    if (Bridge.is($t, System.IDisposable)) {
                        $t.System$IDisposable$Dispose();
                    }
                }



                if (!this.use_last_joints || theta_normed.Count < 2) {

                    var output = System.Linq.Enumerable.from(theta_normed, System.Collections.Generic.List$1(System.Double)).select(function (a) {
                            return a.ToArray();
                        }).ToArray(System.Array.type(System.Double));
                    return output;
                }
                var theta_last = TestGeneralRoboticsToolboxNET.NormalizeJoints.v_builder.Dense$2(joint_index.length);

                for (var i1 = 0; i1 < joint_index.length; i1 = (i1 + 1) | 0) {

                    theta_last.setItem(i1, ($t1 = this.Last_joints)[System.Array.index(joint_index[System.Array.index(i1, joint_index)], $t1)]);
                }

                var theta_dist_arr = System.Array.init(joint_index.length, 0, System.Double);

                var theta_ret = new (System.Collections.Generic.List$1(System.Collections.Generic.List$1(System.Double))).ctor();
                for (var i2 = 0; i2 < joint_index.length; i2 = (i2 + 1) | 0) {

                    var theta_normed_vec = TestGeneralRoboticsToolboxNET.NormalizeJoints.v_builder.DenseOfArray(theta_normed.getItem(i2).ToArray());
                    var theta_dist_vec = theta_normed_vec.Subtract(theta_last.getItem(i2));
                    theta_dist_arr[System.Array.index(i2, theta_dist_arr)] = theta_dist_vec.L2Norm();

                    var passed = true;
                    $t2 = Bridge.getEnumerator(theta_dist_vec, System.Double);
                    try {
                        while ($t2.moveNext()) {
                            var t = $t2.Current;
                            if (!(Math.abs(t) < 1.5707963267948966)) {
                                passed = false;
                            }
                        }
                    } finally {
                        if (Bridge.is($t2, System.IDisposable)) {
                            $t2.System$IDisposable$Dispose();
                        }
                    }
                    if (passed) {
                        theta_ret.add(theta_normed.getItem(i2));
                    }
                }

                /* if (joint_index.Length== 1)
                {

                }
                else
                {*/



                /* foreach(double theta in theta_normed)
                {
                   current_compare = theta; 
                   if (theta_last.ForAll(theta_vec_test)) theta_ret1.Add(theta);
                }*/
                if (theta_ret.Count === 1) {
                    return System.Linq.Enumerable.from(theta_ret, System.Collections.Generic.List$1(System.Double)).select(function (a) {
                            return a.ToArray();
                        }).ToArray(System.Array.type(System.Double));
                }
                if (theta_ret.Count === 0) {
                    theta_ret = theta_normed;
                }
                var index = 0;
                var theta_dist_list = ($t3 = System.Double, System.Linq.Enumerable.from(theta_dist_arr, $t3).toList($t3));
                var returned = new (System.Collections.Generic.List$1(System.Collections.Generic.List$1(System.Double))).ctor();
                for (var z = 0; z < theta_dist_list.Count; z = (z + 1) | 0) {
                    index = theta_dist_list.indexOf(System.Linq.Enumerable.from(theta_dist_list, System.Double).min());
                    returned.setItem(z, theta_normed.getItem(index));
                    theta_dist_list.removeAt(index);
                }
                return System.Linq.Enumerable.from(returned, System.Collections.Generic.List$1(System.Double)).select(function (a) {
                        return a.ToArray();
                    }).ToArray(System.Array.type(System.Double));

            }
        }
    });

    Bridge.define("TestGeneralRoboticsToolboxNET.Robot", {
        fields: {
            H: null,
            P: null,
            Joint_type: null,
            Joint_lower_limit: null,
            Joint_upper_limit: null,
            Joint_vel_limit: null,
            Joint_acc_limit: null,
            M: null,
            R_tool: null,
            P_tool: null,
            Joint_names: null,
            Root_link_name: null,
            Tip_link_name: null
        },
        ctors: {
            ctor: function () {
                this.$initialize();
            },
            $ctor1: function (h, p, joint_type, joint_lower_limit, joint_upper_limit, joint_vel_limit, joint_acc_limit, m, r_tool, p_tool, joint_names, root_link_name, tip_link_name) {
                if (joint_lower_limit === void 0) { joint_lower_limit = null; }
                if (joint_upper_limit === void 0) { joint_upper_limit = null; }
                if (joint_vel_limit === void 0) { joint_vel_limit = null; }
                if (joint_acc_limit === void 0) { joint_acc_limit = null; }
                if (m === void 0) { m = null; }
                if (r_tool === void 0) { r_tool = null; }
                if (p_tool === void 0) { p_tool = null; }
                if (joint_names === void 0) { joint_names = null; }
                if (root_link_name === void 0) { root_link_name = null; }
                if (tip_link_name === void 0) { tip_link_name = null; }
                var $t;

                this.$initialize();
                var AlmostEquals = null;
                var joint_types = System.Array.init([0, 1, 2, 3], System.Int32);
                var splicer = h.EnumerateColumns();

                AlmostEquals = function (val1, val2) {
                    if (Math.abs(val1 - val2) < (Math.pow(10, -8))) {
                        return true;
                    }
                    return false;
                };
                $t = Bridge.getEnumerator(splicer, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double));
                try {
                    while ($t.moveNext()) {
                        var column = $t.Current;
                        if (!(AlmostEquals(column.L2Norm(), 1))) {
                            throw new System.ArgumentException.$ctor1(System.String.format("Matrix H is not Acceptable", null));
                        }
                    }
                } finally {
                    if (Bridge.is($t, System.IDisposable)) {
                        $t.System$IDisposable$Dispose();
                    }
                }
                for (var i = 0; i < joint_type.length; i = (i + 1) | 0) {

                    if (!(System.Array.contains(joint_types, joint_type[System.Array.index(i, joint_type)], System.Int32))) {
                        throw new System.ArgumentException.$ctor1(System.String.format("Joint types contains incorrect values", null));
                    }
                }
                if (h.RowCount !== 3) {
                    throw new System.ArgumentException.$ctor1(System.String.format("Matrix H is not Acceptable", null));
                }
                if (p.RowCount !== 3) {
                    throw new System.ArgumentException.$ctor1(System.String.format("Matrix P is not Acceptable", null));
                }
                if (((h.ColumnCount + 1) | 0) !== p.ColumnCount || h.ColumnCount !== joint_type.length) {
                    throw new System.ArgumentException.$ctor1(System.String.format("Matrix Dimensions are not Acceptable", null));
                }
                if (joint_lower_limit != null && joint_upper_limit != null) {
                    if (joint_lower_limit.length !== joint_type.length || joint_type.length !== joint_upper_limit.length) {
                        throw new System.ArgumentException.$ctor1(System.String.format("Joint Limits not Acceptable", null));
                    }
                    this.Joint_upper_limit = joint_upper_limit;
                    this.Joint_lower_limit = joint_lower_limit;
                } else {
                    this.Joint_lower_limit = null;
                    this.Joint_upper_limit = null;
                }
                if (joint_vel_limit != null) {
                    if (joint_vel_limit.length !== joint_type.length) {
                        throw new System.ArgumentException.$ctor1(System.String.format("Joint Velocities not Acceptable", null));
                    }
                    this.Joint_vel_limit = joint_vel_limit;
                } else {
                    this.Joint_vel_limit = null;
                }
                if (joint_acc_limit != null) {
                    if (joint_acc_limit.length !== joint_type.length) {
                        throw new System.ArgumentException.$ctor1(System.String.format("Joint Accelerations not Acceptable", null));
                    }
                    this.Joint_acc_limit = joint_acc_limit;
                } else {
                    this.Joint_acc_limit = null;
                }
                if (m != null) {
                    if (m.length !== this.H.ColumnCount) {
                        throw new System.ArgumentException.$ctor1(System.String.format("Inertia Matrices not Acceptable", null));
                    }
                    for (var i1 = 0; i1 < m.length; i1 = (i1 + 1) | 0) {
                        if (m[System.Array.index(i1, m)].ColumnCount !== 6 || m[System.Array.index(i1, m)].RowCount !== 6) {
                            throw new System.ArgumentException.$ctor1(System.String.format("Inertia Matrices not Acceptable", null));
                        }
                    }
                    this.M = m;
                } else {
                    this.M = null;
                }
                if (r_tool != null && p_tool != null) {
                    this.R_tool = r_tool;
                    this.P_tool = p_tool;
                } else {
                    this.R_tool = null;
                    this.P_tool = null;
                }
                this.H = h;
                this.P = p;
                this.Joint_type = joint_type;
                if (joint_names != null) {
                    if (joint_names.length !== joint_type.length) {
                        throw new System.ArgumentException.$ctor1(System.String.format("Joint Names not Acceptable", null));
                    }
                    this.Joint_names = joint_names;
                } else {
                    this.Joint_names = null;
                }
                this.Root_link_name = root_link_name;
                this.Tip_link_name = tip_link_name;


            }
        }
    });

    Bridge.define("TestGeneralRoboticsToolboxNET.Transform", {
        statics: {
            methods: {
                op_Multiply: function (tran1, tran2) {
                    var R = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(tran1.R, tran2.R);
                    var P = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(tran1.P, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(tran1.R, tran2.P));
                    var output = new TestGeneralRoboticsToolboxNET.Transform.$ctor1(R, P, tran1.Parent_frame_id, tran2.Child_frame_id);
                    return output;
                },
                op_Equality: function (tran1, tran2) {
                    var AlmostEquals = null;

                    AlmostEquals = function (val1, val2) {
                        if (Math.abs(val1 - val2) < (Math.pow(10, -8))) {
                            return true;
                        }
                        return false;
                    };
                    for (var i = 0; i < 3; i = (i + 1) | 0) {
                        for (var j = 0; i < 3; i = (i + 1) | 0) {
                            if (!(AlmostEquals(tran1.R.getItem(i, j), tran2.R.getItem(i, j)))) {
                                return false;
                            }
                        }
                    }
                    for (var i1 = 0; i1 < 3; i1 = (i1 + 1) | 0) {
                        if (!(AlmostEquals(tran1.P.getItem(i1), tran2.P.getItem(i1)))) {
                            return false;
                        }
                    }
                    return true;

                },
                op_Inequality: function (tran1, tran2) {
                    return (!(TestGeneralRoboticsToolboxNET.Transform.op_Equality(tran1, tran2)));
                }
            }
        },
        fields: {
            R: null,
            P: null,
            Parent_frame_id: null,
            Child_frame_id: null
        },
        ctors: {
            ctor: function () {
                this.$initialize();
            },
            $ctor1: function (r, p, parent_frame_id, child_frame_id) {
                if (parent_frame_id === void 0) { parent_frame_id = null; }
                if (child_frame_id === void 0) { child_frame_id = null; }

                this.$initialize();
                if (r.ColumnCount !== 3 || r.RowCount !== 3) {
                    throw new System.ArgumentException.$ctor1(System.String.format("Rotation Matrix for transform not Acceptable", null));
                }
                if (p.Count !== 3) {
                    throw new System.ArgumentException.$ctor1(System.String.format("Position Vector for transform not Acceptable", null));
                }
                this.R = r;
                this.P = p;
                this.Parent_frame_id = parent_frame_id;
                this.Child_frame_id = child_frame_id;
            }
        },
        methods: {
            inv: function (tran1) {
                var R = tran1.R.Transpose();
                var P = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_UnaryNegation((MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(tran1.R, tran1.P)));
                var output = new TestGeneralRoboticsToolboxNET.Transform.$ctor1(R, P, tran1.Parent_frame_id, tran1.Child_frame_id);
                return output;
            }
        }
    });
});

//# sourceMappingURL=data:application/json;base64,ewogICJ2ZXJzaW9uIjogMywKICAiZmlsZSI6ICJCcmlkZ2VHZW5lcmFsUm9ib3RpY3NUb29sYm94LmpzIiwKICAic291cmNlUm9vdCI6ICIiLAogICJzb3VyY2VzIjogWyJBcHAuY3MiLCJDbGFzczEuY3MiLCJJbnZlcnNlS2luLmNzIl0sCiAgIm5hbWVzIjogWyIiXSwKICAibWFwcGluZ3MiOiAiOzs7Ozs7Ozs7OztZQWdCWUE7Ozs7Ozs7Ozs7Ozs7Ozs7cUNDRXFDQTtxQ0FDQUE7Ozs7K0JBRVJBO29CQUU3QkEsV0FBc0JBO29CQUN0QkEsbUJBQWFBLENBQUNBO29CQUNkQSxtQkFBYUE7b0JBQ2JBLG1CQUFhQTtvQkFDYkEsbUJBQWFBLENBQUNBO29CQUNkQSxtQkFBYUEsQ0FBQ0E7b0JBQ2RBLG1CQUFhQTtvQkFDYkEsT0FBT0E7O2tDQUd5QkE7b0JBRWhDQTt3QkFBaUJBLENBQUNBLENBQUNBLHFCQUFhQTt3QkFBYUEsQ0FBQ0EscUJBQWFBO3dCQUFhQSxDQUFDQSxDQUFDQSxxQkFBYUE7O29CQUN2RkEsYUFBd0JBLDRFQUF1QkE7b0JBQy9DQTtvQkFDQUEsT0FBT0E7OytCQUVzQkEsR0FBa0JBO29CQUUvQ0EsUUFBbUJBO29CQUNuQkEsV0FBc0JBLHlEQUFJQTtvQkFDMUJBLFlBQXVCQSxnQkFBY0E7b0JBQ3JDQSxPQUFPQSxDQUFDQSw2SUFBSUEsNEVBQVNBLFFBQVNBLFFBQU9BLG9FQUFDQSxNQUFNQSxTQUFTQSxTQUFVQTs7aUNBRWpCQTtvQkFFOUNBLFNBQW9CQSwyRUFBSUE7b0JBRXhCQSxnQkFBbUJBO29CQUNuQkEsZ0JBQW1CQSxDQUFDQTtvQkFDcEJBLFlBQWVBLFdBQVdBLFdBQVdBO29CQUNyQ0E7b0JBQ0FBLElBQUlBLFlBQVlBO3dCQUVaQSxJQUFJQTs0QkFFQUEsSUFBSUEsNEVBQXVCQTs7NEJBRTNCQSxPQUFPQSxTQUFrQ0E7OzRCQUl6Q0EsVUFBcUJBOzRCQUNyQkEsUUFBbUJBLG9FQUFDQSxNQUFhQSxDQUFDQSx3RUFBSUE7NEJBQ3RDQSxJQUFJQSw0RUFBdUJBLG1CQUFR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R0EsZ0JBQVFBLEdBQUdBO2dDQUFNQTs7OztvQkFHMURBLEtBQUtBLFlBQVdBLFFBQU9BO3dCQUVuQkEsSUFBSUEsQ0FBQ0EsQ0FBQ0EsYUFBYUEsZ0JBQVFBLEtBQUlBLGdCQUFRQTs0QkFBTUE7OztvQkFFakRBOzs7eUNBRzJCQSxPQUFpQkE7b0JBRTVDQSxPQUFPQSxDQUFDQSxDQUFDQSxDQUFDQSwyREFBU0E7Ozs7Ozs7Ozs7Ozs7OzhCQTVDTkEsR0FBa0JBLEdBQWtCQSxpQkFBMENBOzs7OztnQkFFM0ZBLElBQUlBLHVCQUFzQkE7b0JBQWlCQSxNQUFNQSxJQUFJQSxnQ0FBa0JBOztnQkFDdkVBLElBQUdBO29CQUFZQSxNQUFNQSxJQUFJQSxnQ0FBa0JBOztnQkFDM0NBLFNBQUlBO2dCQUNKQSxTQUFJQTtnQkFDSkEsdUJBQWtCQTtnQkFDbEJBLHNCQUFpQkE7Ozs7MkJBdUNBQTtnQkFFakJBLFFBQW1CQTtnQkFDbkJBLFFBQW1CQSx3RUFBQ0EsQ0FBQ0EsOEVBQVFBO2dCQUM3QkEsYUFBbUJBLElBQUlBLCtDQUFVQSxHQUFHQSxHQUFHQSx1QkFBdUJBO2dCQUM5REEsT0FBT0EiLAogICJzb3VyY2VzQ29udGVudCI6IFsidXNpbmcgQnJpZGdlO1xyXG51c2luZyBOZXd0b25zb2Z0Lkpzb247XHJcbnVzaW5nIEJyaWRnZS5IdG1sNTtcclxudXNpbmcgQnJpZGdlR2VuZXJhbFJvYm90aWNzVG9vbGJveDtcclxudXNpbmcgR2VuZXJhbFJvYm90aWNzVG9vbGJveFRlc3RzO1xyXG51c2luZyBTeXN0ZW07XHJcblxyXG5cclxubmFtZXNwYWNlIEJyaWRnZUdlbmVyYWxSb2JvdGljc1Rvb2xib3hcclxue1xyXG4gICAgcHVibGljIGNsYXNzIEFwcFxyXG4gICAge1xyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgdm9pZCBNYWluKClcclxuICAgICAgICB7ICAgIFxyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKFwiV2VsY29tZSB0byBCcmlkZ2UuTkVUXCIpO1xyXG4gICAgICAgICAgICBcclxuICAgICAgICAgICAgVW5pdFRlc3QxLlJ1blRlc3RzKCk7XHJcblxyXG4gICAgICAgICAgICAvLyBBZnRlciBidWlsZGluZyAoQ3RybCArIFNoaWZ0ICsgQikgdGhpcyBwcm9qZWN0LCBcclxuICAgICAgICAgICAgLy8gYnJvd3NlIHRvIHRoZSAvYmluL0RlYnVnIG9yIC9iaW4vUmVsZWFzZSBmb2xkZXIuXHJcblxyXG4gICAgICAgICAgICAvLyBBIG5ldyBicmlkZ2UvIGZvbGRlciBoYXMgYmVlbiBjcmVhdGVkIGFuZFxyXG4gICAgICAgICAgICAvLyBjb250YWlucyB5b3VyIHByb2plY3RzIEphdmFTY3JpcHQgZmlsZXMuIFxyXG5cclxuICAgICAgICAgICAgLy8gT3BlbiB0aGUgYnJpZGdlL2luZGV4Lmh0bWwgZmlsZSBpbiBhIGJyb3dzZXIgYnlcclxuICAgICAgICAgICAgLy8gUmlnaHQtQ2xpY2sgPiBPcGVuIFdpdGguLi4sIHRoZW4gY2hvb3NlIGFcclxuICAgICAgICAgICAgLy8gd2ViIGJyb3dzZXIgZnJvbSB0aGUgbGlzdFxyXG5cclxuICAgICAgICAgICAgLy8gVGhpcyBhcHBsaWNhdGlvbiB3aWxsIHRoZW4gcnVuIGluIHRoZSBicm93c2VyLlxyXG4gICAgICAgIH1cclxuICAgIH1cclxufSIsInVzaW5nIFN5c3RlbTtcclxudXNpbmcgU3lzdGVtLkNvbGxlY3Rpb25zLkdlbmVyaWM7XHJcbnVzaW5nIFN5c3RlbS5MaW5xO1xyXG51c2luZyBTeXN0ZW0uRGlhZ25vc3RpY3M7XHJcbnVzaW5nIFN5c3RlbS5UZXh0O1xyXG4vL3VzaW5nIFN5c3RlbS5OdW1lcmljcztcclxudXNpbmcgU3lzdGVtLlRocmVhZGluZy5UYXNrcztcclxudXNpbmcgTWF0aE5ldC5OdW1lcmljcztcclxudXNpbmcgTWF0aE5ldC5OdW1lcmljcy5MaW5lYXJBbGdlYnJhO1xyXG51c2luZyBNYXRoTmV0Lk51bWVyaWNzLkxpbmVhckFsZ2VicmEuRG91YmxlO1xyXG51c2luZyBNYXRoTmV0Lk51bWVyaWNzLkxpbmVhckFsZ2VicmEuRG91YmxlLk1hdGhOZXQuTnVtZXJpY3MuTGluZWFyQWxnZWJyYTtcclxudXNpbmcgTWF0aE5ldC5OdW1lcmljcy5MaW5lYXJBbGdlYnJhLkZhY3Rvcml6YXRpb247XHJcbnVzaW5nIE1hdGhOZXQuTnVtZXJpY3MuTGluZWFyQWxnZWJyYS5Eb3VibGUuRmFjdG9yaXphdGlvbjtcclxuXHJcbm5hbWVzcGFjZSBUZXN0R2VuZXJhbFJvYm90aWNzVG9vbGJveE5FVFxyXG57XHJcbiAgICBwdWJsaWMgY2xhc3MgR2VuZXJhbFJvYm90aWNzVG9vbGJveFxyXG4gICAge1xyXG4gICAgICAgIHN0YXRpYyBWZWN0b3JCdWlsZGVyPGRvdWJsZT4gdl9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uVmVjdG9yO1xyXG4gICAgICAgIHN0YXRpYyBNYXRyaXhCdWlsZGVyPGRvdWJsZT4gbV9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uTWF0cml4O1xyXG4gICAgICAgIHB1YmxpYyBHZW5lcmFsUm9ib3RpY3NUb29sYm94KCkgeyB9XHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBNYXRyaXg8ZG91YmxlPiBIYXQoVmVjdG9yPGRvdWJsZT4gaylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IGtoYXQgPSBtX2J1aWxkZXIuRGVuc2UoMywgMyk7XHJcbiAgICAgICAgICAgIGtoYXRbMCwgMV0gPSAta1syXTtcclxuICAgICAgICAgICAga2hhdFswLCAyXSA9IGtbMV07XHJcbiAgICAgICAgICAgIGtoYXRbMSwgMF0gPSBrWzJdO1xyXG4gICAgICAgICAgICBraGF0WzEsIDJdID0gLWtbMF07XHJcbiAgICAgICAgICAgIGtoYXRbMiwgMF0gPSAta1sxXTtcclxuICAgICAgICAgICAga2hhdFsyLCAxXSA9IGtbMF07XHJcbiAgICAgICAgICAgIHJldHVybiBraGF0O1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBWZWN0b3I8ZG91YmxlPiBJbnZoYXQoTWF0cml4PGRvdWJsZT4ga2hhdClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGludiA9IHsgKC1raGF0WzEsIDJdICsga2hhdFsyLCAxXSksIChraGF0WzAsIDJdIC0ga2hhdFsyLCAwXSksICgta2hhdFswLCAxXSArIGtoYXRbMSwgMF0pIH07XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IG91dHB1dCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkoaW52KTtcclxuICAgICAgICAgICAgb3V0cHV0IC89IDI7XHJcbiAgICAgICAgICAgIHJldHVybiBvdXRwdXQ7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgTWF0cml4PGRvdWJsZT4gUm90KFZlY3Rvcjxkb3VibGU+IGssIGRvdWJsZSB0aGV0YSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IEkgPSBtX2J1aWxkZXIuRGVuc2VJZGVudGl0eSgzKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4ga2hhdCA9IEhhdChrKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4ga2hhdDIgPSBraGF0Lk11bHRpcGx5KGtoYXQpO1xyXG4gICAgICAgICAgICByZXR1cm4gKEkgKyBNYXRoLlNpbih0aGV0YSkgKiBraGF0ICsgKDEuMCAtIE1hdGguQ29zKHRoZXRhKSkgKiBraGF0Mik7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgVHVwbGU8VmVjdG9yPGRvdWJsZT4sIGRvdWJsZT4gUjJyb3QoTWF0cml4PGRvdWJsZT4gUilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIxID0gUiAtIFIuVHJhbnNwb3NlKCk7XHJcbiAgICAgICAgICAgIC8vZG91YmxlIHNpbl90aGV0YSA9IFIxLkwyTm9ybSgpIC8gTWF0aC5TcXJ0KDgpO1xyXG4gICAgICAgICAgICBkb3VibGUgc2luX3RoZXRhID0gUjEuTDJOb3JtKCkgLyAyLjA7XHJcbiAgICAgICAgICAgIGRvdWJsZSBjb3NfdGhldGEgPSAoUi5UcmFjZSgpIC0gMS4wKSAvIDIuMDtcclxuICAgICAgICAgICAgZG91YmxlIHRoZXRhID0gTWF0aC5BdGFuMihzaW5fdGhldGEsIGNvc190aGV0YSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGs7XHJcbiAgICAgICAgICAgIGlmIChzaW5fdGhldGEgPCAoMWUtNikpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChjb3NfdGhldGEgPiAwKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDEuMCB9KTtcclxuXHJcbiAgICAgICAgICAgICAgICAgICAgcmV0dXJuIG5ldyBUdXBsZTxWZWN0b3I8ZG91YmxlPiwgZG91YmxlPihrLCAwKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBleWUgPSBtX2J1aWxkZXIuRGVuc2VJZGVudGl0eSgzKTtcclxuICAgICAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBCID0gKDEuMCAvIDIuMCkgKiAoUiArIGV5ZSk7XHJcbiAgICAgICAgICAgICAgICAgICAgayA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyBNYXRoLlNxcnQoQlswLCAwXSksIE1hdGguU3FydChCWzEsIDFdKSwgTWF0aC5TcXJ0KEJbMiwgMl0pIH0pO1xyXG4gICAgICAgICAgICAgICAgICAgIGlmIChNYXRoLkFicyhrWzBdKSA+IDFlLTYpXHJcbiAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBrWzFdID0ga1sxXSAqIE1hdGguU2lnbihCWzAsIDFdIC8ga1swXSk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIGtbMl0gPSBrWzJdICogTWF0aC5TaWduKEJbMCwgMl0gLyBrWzBdKTtcclxuICAgICAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICAgICAgZWxzZSBpZiAoTWF0aC5BYnMoa1sxXSkgPiAxZS02KVxyXG4gICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAga1syXSA9IGtbMl0gKiBNYXRoLlNpZ24oQlswLCAyXSAvIGtbMV0pO1xyXG4gICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgICByZXR1cm4gbmV3IFR1cGxlPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+KGssIE1hdGguUEkpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDAuMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gaW52ID0gSW52aGF0KFIxKTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCBrLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGtbaV0gPSBpbnZbaV0gLyAoMi4wICogc2luX3RoZXRhKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFR1cGxlPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+KGssIHRoZXRhKTtcclxuXHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgTWF0cml4PGRvdWJsZT4gU2NyZXdfbWF0cml4KFZlY3Rvcjxkb3VibGU+IHIpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBJID0gbV9idWlsZGVyLkRlbnNlSWRlbnRpdHkoNik7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IGhhdCA9IEhhdChyKTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCAzOyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGZvciAoaW50IGogPSAzOyBqIDwgNjsgaisrKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIElbaSwgal0gPSBoYXRbaSwgaiAtIDNdO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIHJldHVybiBJO1xyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIE1hdHJpeDxkb3VibGU+IFEyUihWZWN0b3I8ZG91YmxlPiBxKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gSSA9IG1fYnVpbGRlci5EZW5zZUlkZW50aXR5KDMpO1xyXG5cclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gaGF0ID0gSGF0KHEuU3ViVmVjdG9yKDEsIDMpKTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHFoYXQyID0gaGF0Lk11bHRpcGx5KGhhdCk7XHJcbiAgICAgICAgICAgIHJldHVybiBJICsgMiAqIHFbMF0gKiBoYXQgKyAyICogcWhhdDI7XHJcblxyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIFZlY3Rvcjxkb3VibGU+IFIyUShNYXRyaXg8ZG91YmxlPiBSKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgZG91YmxlIHRyID0gUi5UcmFjZSgpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxO1xyXG4gICAgICAgICAgICBpZiAodHIgPiAwKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBkb3VibGUgUyA9IDIgKiBNYXRoLlNxcnQodHIgKyAxKTtcclxuXHJcbiAgICAgICAgICAgICAgICBxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7KDAuMjUgKiBTKSxcclxuICAgICAgICAgICAgICAgICAgICAgICgoUlsyLCAxXSAtIFJbMSwgMl0pIC8gUyksXHJcbiAgICAgICAgICAgICAgICAgICAgICAoKFJbMCwgMl0gLSBSWzIsIDBdKSAvIFMpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgKChSWzEsIDBdIC0gUlswLCAxXSkgLyBTKX0pO1xyXG5cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlIGlmIChSWzAsIDBdID4gUlsxLCAxXSAmJiBSWzAsIDBdID4gUlsyLCAyXSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIFMgPSAyICogTWF0aC5TcXJ0KDEgKyBSWzAsIDBdIC0gUlsxLCAxXSAtIFJbMiwgMl0pO1xyXG4gICAgICAgICAgICAgICAgcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geygoUlsyLCAxXSAtIFJbMSwgMl0pIC8gUyksXHJcbiAgICAgICAgICAgICAgICAgICAgICAoMC4yNSAqIFMpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgKChSWzAsIDFdICsgUlsxLCAwXSkgLyBTKSxcclxuICAgICAgICAgICAgICAgICAgICAgICgoUlswLCAyXSArIFJbMiwgMF0pIC8gUyl9KTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlIGlmIChSWzEsIDFdID4gUlsyLCAyXSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIFMgPSAyICogTWF0aC5TcXJ0KDEgLSBSWzAsIDBdICsgUlsxLCAxXSAtIFJbMiwgMl0pO1xyXG4gICAgICAgICAgICAgICAgcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geygoUlswLCAyXSAtIFJbMiwgMF0pIC8gUyksXHJcbiAgICAgICAgICAgICAgICAgICAgICAoKFJbMCwgMV0gKyBSWzEsIDBdKSAvIFMpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgKDAuMjUgKiBTKSxcclxuICAgICAgICAgICAgICAgICAgICAgICgoUlsxLCAyXSArIFJbMiwgMV0pIC8gUykgfSk7XHJcblxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIFMgPSAyICogTWF0aC5TcXJ0KDEgLSBSWzAsIDBdIC0gUlsxLCAxXSArIFJbMiwgMl0pO1xyXG4gICAgICAgICAgICAgICAgcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geygoUlsxLCAwXSAtIFJbMCwgMV0pIC8gUyksXHJcbiAgICAgICAgICAgICAgICAgICAgICAoKFJbMCwgMl0gKyBSWzIsIDBdKSAvIFMpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgKChSWzEsIDJdICsgUlsyLCAxXSkgLyBTKSxcclxuICAgICAgICAgICAgICAgICAgICAgICgwLjI1ICogUykgfSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgcmV0dXJuIHE7XHJcblxyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIFR1cGxlPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+IFEyUm90KFZlY3Rvcjxkb3VibGU+IHEpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBkb3VibGUgdGhldGEgPSAyICogTWF0aC5BY29zKHFbMF0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrO1xyXG4gICAgICAgICAgICBpZiAoTWF0aC5BYnModGhldGEpIDwgKE1hdGguUG93KDEwLC02KSkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDEuMCB9KTtcclxuXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gbmV3IFR1cGxlPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+KGssIDApO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDEuMCB9KTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDE7IGkgPCBrLkNvdW50ICsgMTsgaSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBrW2kgLSAxXSA9IHFbaV0gLyBNYXRoLlNpbih0aGV0YSAvIDIuMCk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBUdXBsZTxWZWN0b3I8ZG91YmxlPiwgZG91YmxlPihrLCB0aGV0YSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgVmVjdG9yPGRvdWJsZT4gUm90MlEoZG91YmxlW10gaywgZG91YmxlIHRoZXRhKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gb3V0cHV0ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IE1hdGguQ29zKHRoZXRhLzIuMCksIGtbMF0qTWF0aC5TaW4odGhldGEvMi4wKSwga1sxXSAqIE1hdGguU2luKHRoZXRhIC8gMi4wKSwga1syXSAqIE1hdGguU2luKHRoZXRhIC8gMi4wKSB9KTtcclxuICAgICAgICAgICAgcmV0dXJuIG91dHB1dDtcclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBWZWN0b3I8ZG91YmxlPiBRdWF0Y29tcGxlbWVudChWZWN0b3I8ZG91YmxlPiBxKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gb3V0cHV0ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IHFbMF0sIC0xICogcVsxXSwgLTEgKiBxWzJdLCAtMSAqIHFbM10gfSk7XHJcbiAgICAgICAgICAgIHJldHVybiBvdXRwdXQ7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgTWF0cml4PGRvdWJsZT4gUXVhdHByb2R1Y3QoVmVjdG9yPGRvdWJsZT4gcSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IEkgPSBtX2J1aWxkZXIuRGVuc2VJZGVudGl0eSgzKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUSA9IG1fYnVpbGRlci5EZW5zZSg0LCA0KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gaGF0cyA9IEhhdChxLlN1YlZlY3RvcigxLCAzKSk7XHJcbiAgICAgICAgICAgIFFbMCwgMF0gPSBxWzBdO1xyXG5cclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDE7IGkgPCA0OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIFFbMCwgaV0gPSAtcVtpXTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBmb3IgKGludCBpID0gMTsgaSA8IDQ7IGkrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgUVtpLCAwXSA9IHFbaV07XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDE7IGkgPCA0OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGZvciAoaW50IGogPSAxOyBqIDwgNDsgaisrKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIFFbaSwgal0gPSBxWzBdICogSVtpIC0gMSwgaiAtIDFdICsgaGF0c1tpIC0gMSwgaiAtIDFdO1xyXG4gICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm4gUTtcclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBNYXRyaXg8ZG91YmxlPiBRdWF0amFjb2JpYW4oVmVjdG9yPGRvdWJsZT4gcSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IEkgPSBtX2J1aWxkZXIuRGVuc2VJZGVudGl0eSgzKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gSiA9IG1fYnVpbGRlci5EZW5zZSg0LCAzKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gaGF0cyA9IEhhdChxLlN1YlZlY3RvcigxLCAzKSk7XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgMzsgaSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBKWzAsIGldID0gMC41ICogLXFbaSArIDFdO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAxOyBpIDwgNDsgaSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBmb3IgKGludCBqID0gMDsgaiA8IDM7IGorKylcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBKW2ksIGpdID0gMC41ICogKHFbMF0gKiBJW2kgLSAxLCBqXSAtIGhhdHNbaSAtIDEsIGpdKTtcclxuICAgICAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgcmV0dXJuIEo7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgTWF0cml4PGRvdWJsZT4gUnB5MlIoVmVjdG9yPGRvdWJsZT4gcnB5KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gaztcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90YXRpb24xID0gUm90KGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDEuMCB9KSwgcnB5WzJdKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90YXRpb24yID0gUm90KGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAxLjAsIDAuMCB9KSwgcnB5WzFdKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90YXRpb24zID0gUm90KGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAwLjAsIDAuMCB9KSwgcnB5WzBdKTtcclxuICAgICAgICAgICAgcmV0dXJuIHJvdGF0aW9uMS5NdWx0aXBseShyb3RhdGlvbjIpLk11bHRpcGx5KHJvdGF0aW9uMyk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgVmVjdG9yPGRvdWJsZT4gUjJScHkoTWF0cml4PGRvdWJsZT4gUilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIC8vaWYoUi5Db2x1bW4oMCkuTDJOb3JtKCkpXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IG91dHB1dDtcclxuICAgICAgICAgICAgZG91YmxlIHIgPSBNYXRoLkF0YW4yKFJbMiwgMV0sIFJbMiwgMl0pO1xyXG4gICAgICAgICAgICBkb3VibGUgeSA9IE1hdGguQXRhbjIoUlsxLCAwXSwgUlswLCAwXSk7XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICBkb3VibGUgbm9ybWllID0gKFIuU3ViTWF0cml4KDIsIDEsIDEsIDIpKS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgZG91YmxlIHAgPSBNYXRoLkF0YW4yKC1SWzIsIDBdLCBub3JtaWUpO1xyXG4gICAgICAgICAgICByZXR1cm4gb3V0cHV0ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IHIsIHAsIHkgfSk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBwdWJsaWMgc3RhdGljIFRyYW5zZm9ybSBGd2RraW4oUm9ib3Qgcm9ib3QsIGRvdWJsZVtdIHRoZXRhKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIGlmKHJvYm90LkpvaW50X2xvd2VyX2xpbWl0IT1udWxsICYmIHJvYm90LkpvaW50X3VwcGVyX2xpbWl0ICE9IG51bGwpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGZvcihpbnQgaSA9IDA7IGkgPCB0aGV0YS5MZW5ndGg7IGkrKylcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBpZighKHJvYm90LkpvaW50X2xvd2VyX2xpbWl0W2ldPHRoZXRhW2ldICYmIHRoZXRhW2ldPHJvYm90LkpvaW50X3VwcGVyX2xpbWl0W2ldKSkgdGhyb3cgbmV3IEFyZ3VtZW50RXhjZXB0aW9uKFN0cmluZy5Gb3JtYXQoXCJKb2ludCBhbmdsZSBmb3Igam9pbnQgezB9IG91dCBvZiByYW5nZVwiLGkpKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBwO1xyXG4gICAgICAgICAgICBwID0gcm9ib3QuUC5Db2x1bW4oMCk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIgPSBtX2J1aWxkZXIuRGVuc2VJZGVudGl0eSgzKTtcclxuICAgICAgICAgICAgZm9yKGludCBpID0gMDsgaSA8IHJvYm90LkpvaW50X3R5cGUuTGVuZ3RoOyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmKHJvYm90LkpvaW50X3R5cGVbaV09PTAgfHwgcm9ib3QuSm9pbnRfdHlwZVtpXSA9PSAyKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIFIgPSBSICogUm90KHJvYm90LkguQ29sdW1uKGkpLCB0aGV0YVtpXSk7XHJcbiAgICAgICAgICAgICAgICB9ZWxzZSBpZihyb2JvdC5Kb2ludF90eXBlW2ldID09IDEgfHwgcm9ib3QuSm9pbnRfdHlwZVtpXSA9PSAzKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIHAgPSBwICsgdGhldGFbaV0gKiBSICogcm9ib3QuSC5Db2x1bW4oaSk7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBwID0gcCArIFIgKiByb2JvdC5QLkNvbHVtbihpICsgMSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIC8vUmVzaGFwZSBoZXJlIG5vdCBhZGRlZFxyXG4gICAgICAgICAgICBpZiAocm9ib3QuUl90b29sICE9IG51bGwgJiYgcm9ib3QuUF90b29sICE9IG51bGwpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIFxyXG4gICAgICAgICAgICAgICAgcCA9IHAgKyBSLk11bHRpcGx5KHJvYm90LlBfdG9vbCk7XHJcbiAgICAgICAgICAgICAgICBSID0gUipyb2JvdC5SX3Rvb2w7XHJcbiAgICAgICAgICAgICAgICBcclxuICAgICAgICAgICAgICAgIC8vcCA9IHAgKyBSKnJvYm90LlBfdG9vbDtcclxuICAgICAgICAgICAgICAgIC8vUiA9IFIqcm9ib3QuUl90b29sO1xyXG5cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBUcmFuc2Zvcm0gb3V0cHV0ID0gbmV3IFRyYW5zZm9ybShSLCBwKTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIHJldHVybiBvdXRwdXQ7XHJcblxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBNYXRyaXg8ZG91YmxlPiBSb2JvdGphY29iaWFuKFJvYm90IHJvYm90LCBkb3VibGVbXSB0aGV0YSlcclxuICAgICAgICB7XHJcblxyXG4gICAgICAgICAgICBpZiAocm9ib3QuSm9pbnRfbG93ZXJfbGltaXQgIT0gbnVsbCAmJiByb2JvdC5Kb2ludF91cHBlcl9saW1pdCAhPSBudWxsKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBmb3IgKGludCBrID0gMDsgayA8IHRoZXRhLkxlbmd0aDsgaysrKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGlmICghKHJvYm90LkpvaW50X2xvd2VyX2xpbWl0W2tdIDwgdGhldGFba10gJiYgdGhldGFba10gPCByb2JvdC5Kb2ludF91cHBlcl9saW1pdFtrXSkpIHRocm93IG5ldyBBcmd1bWVudEV4Y2VwdGlvbihTdHJpbmcuRm9ybWF0KFwiSm9pbnQgYW5nbGVzIG91dCBvZiByYW5nZVwiKSk7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IGhpID0gbV9idWlsZGVyLkRlbnNlKHJvYm90LkguUm93Q291bnQsIHJvYm90LkguQ29sdW1uQ291bnQpO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBwMGkgPSBtX2J1aWxkZXIuRGVuc2Uocm9ib3QuUC5Sb3dDb3VudCwgcm9ib3QuUC5Db2x1bW5Db3VudCk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHA7XHJcbiAgICAgICAgICAgIHAgPSByb2JvdC5QLkNvbHVtbigwKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUiA9IG1fYnVpbGRlci5EZW5zZUlkZW50aXR5KDMpO1xyXG5cclxuICAgICAgICAgICAgcDBpLlNldENvbHVtbigwLHApO1xyXG4gICAgICAgICAgICBcclxuXHJcblxyXG4gICAgICAgICAgICBmb3IgKGludCBrID0gMDsgayA8IHJvYm90LkpvaW50X3R5cGUuTGVuZ3RoOyBrKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChyb2JvdC5Kb2ludF90eXBlW2tdID09IDAgfHwgcm9ib3QuSm9pbnRfdHlwZVtrXSA9PSAyKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIFIgPSBSICogUm90KHJvYm90LkguQ29sdW1uKGspLCB0aGV0YVtrXSk7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBlbHNlIGlmIChyb2JvdC5Kb2ludF90eXBlW2tdID09IDEgfHwgcm9ib3QuSm9pbnRfdHlwZVtrXSA9PSAzKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIHAgPSBwICsgdGhldGFba10gKiBSICogcm9ib3QuSC5Db2x1bW4oayk7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBwID0gcCArIFIgKiByb2JvdC5QLkNvbHVtbihrICsgMSk7XHJcbiAgICAgICAgICAgICAgICBwMGkuU2V0Q29sdW1uKGsgKyAxLCBwKTtcclxuICAgICAgICAgICAgICAgIGhpLlNldENvbHVtbihrLCBSICogcm9ib3QuSC5Db2x1bW4oaykpO1xyXG5cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBwMFQgPSBwMGkuQ29sdW1uKHJvYm90LkpvaW50X3R5cGUuTGVuZ3RoKTtcclxuICAgICAgICAgICAgXHJcblxyXG4gICAgICAgICAgICBpZiAocm9ib3QuUF90b29sICE9IG51bGwpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHAwVCA9IHAwVCArIFIgKiByb2JvdC5QX3Rvb2w7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gSj0gbV9idWlsZGVyLkRlbnNlKDYsIHJvYm90LkpvaW50X3R5cGUuTGVuZ3RoKTtcclxuICAgICAgICAgICAgaW50IGkgPSAwO1xyXG4gICAgICAgICAgICBpbnQgaiA9IDA7XHJcbiAgICAgICAgICAgIHdoaWxlIChpIDwgcm9ib3QuSm9pbnRfdHlwZS5MZW5ndGgpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChyb2JvdC5Kb2ludF90eXBlW2ldID09IDApXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgSi5TZXRDb2x1bW4oaiwgMCwgMywgaGkuQ29sdW1uKGkpKTtcclxuICAgICAgICAgICAgICAgICAgICBKLlNldENvbHVtbihqLCAzLCAzLCBIYXQoaGkuQ29sdW1uKGkpKSAqIChwMFQgLSBwMGkuQ29sdW1uKGkpKSk7XHJcbiAgICAgICAgICAgICAgICB9ZWxzZSBpZihyb2JvdC5Kb2ludF90eXBlW2ldID09IDEpXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgSi5TZXRDb2x1bW4oaiwgMywgMywgaGkuQ29sdW1uKGkpKTtcclxuICAgICAgICAgICAgICAgIH1lbHNlIGlmKHJvYm90LkpvaW50X3R5cGVbaV0gPT0gMylcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBKLlNldENvbHVtbihqLCAzLCAzLCBSb3QoaGkuQ29sdW1uKGkgKyAyKSwgdGhldGFbaSArIDJdKSAqIGhpLkNvbHVtbihpKSk7XHJcbiAgICAgICAgICAgICAgICAgICAgSi5TZXRDb2x1bW4oaiArIDEsIDAsIDMsIGhpLkNvbHVtbihpICsgMikpO1xyXG4gICAgICAgICAgICAgICAgICAgIEouU2V0Q29sdW1uKGorMSwgMywgMywgSGF0KGhpLkNvbHVtbihpKzIpKSAqIChwMFQgLSBwMGkuQ29sdW1uKGkrMikpKTtcclxuICAgICAgICAgICAgICAgICAgICBKID0gSi5TdWJNYXRyaXgoMCwgSi5Sb3dDb3VudCwgMCwgSi5Db2x1bW5Db3VudCAtIDEpO1xyXG4gICAgICAgICAgICAgICAgICAgIGkrPTI7XHJcbiAgICAgICAgICAgICAgICAgICAgaisrO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgaSsrO1xyXG4gICAgICAgICAgICAgICAgaisrO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIHJldHVybiBKO1xyXG5cclxuXHJcbiAgICAgICAgICAgIFxyXG5cclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBWZWN0b3I8ZG91YmxlPiBDcm9zcyhWZWN0b3I8ZG91YmxlPiBsZWZ0LCBWZWN0b3I8ZG91YmxlPiByaWdodClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICgobGVmdC5Db3VudCAhPSAzIHx8IHJpZ2h0LkNvdW50ICE9IDMpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBzdHJpbmcgbWVzc2FnZSA9IFwiVmVjdG9ycyBtdXN0IGhhdmUgYSBsZW5ndGggb2YgMy5cIjtcclxuICAgICAgICAgICAgICAgIHRocm93IG5ldyBFeGNlcHRpb24obWVzc2FnZSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcmVzdWx0ID0gIHZfYnVpbGRlci5EZW5zZSgzKTtcclxuICAgICAgICAgICAgcmVzdWx0WzBdID0gbGVmdFsxXSAqIHJpZ2h0WzJdIC0gbGVmdFsyXSAqIHJpZ2h0WzFdO1xyXG4gICAgICAgICAgICByZXN1bHRbMV0gPSAtbGVmdFswXSAqIHJpZ2h0WzJdICsgbGVmdFsyXSAqIHJpZ2h0WzBdO1xyXG4gICAgICAgICAgICByZXN1bHRbMl0gPSBsZWZ0WzBdICogcmlnaHRbMV0gLSBsZWZ0WzFdICogcmlnaHRbMF07XHJcblxyXG4gICAgICAgICAgICByZXR1cm4gcmVzdWx0O1xyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGRvdWJsZSBTdWJwcm9ibGVtMChWZWN0b3I8ZG91YmxlPiBwLCBWZWN0b3I8ZG91YmxlPiBxLCBWZWN0b3I8ZG91YmxlPiBrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgZG91YmxlIG1pbiA9IGRvdWJsZS5FcHNpbG9uO1xyXG4gICAgICAgICAgICBpZiAoIShrICogcCA8IG1pbiAmJiBrICogcSA8IG1pbikpIHRocm93IG5ldyBBcmd1bWVudEV4Y2VwdGlvbihTdHJpbmcuRm9ybWF0KFwiayBtdXN0IGJlIHBlcnBlbmRpY3VsYXIgdG8gcCBhbmQgcVwiKSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGVwID0gcCAvIHAuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGVxID0gcSAvIHEuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIGRvdWJsZSB0aGV0YSA9IDIgKiBNYXRoLkF0YW4yKChlcCAtIGVxKS5MMk5vcm0oKSwgKGVwICsgZXEpLkwyTm9ybSgpKTtcclxuICAgICAgICAgICAgaWYgKGsgKiBDcm9zcyhwLCBxKSA8IDApIHJldHVybiAtdGhldGE7XHJcbiAgICAgICAgICAgIHJldHVybiB0aGV0YTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgZG91YmxlIFN1YnByb2JsZW0xKFZlY3Rvcjxkb3VibGU+IHAsIFZlY3Rvcjxkb3VibGU+IHEsIFZlY3Rvcjxkb3VibGU+IGspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBkb3VibGUgbWluID0gZG91YmxlLkVwc2lsb247XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IG1pbnVzID0gcCAtIHE7XHJcbiAgICAgICAgICAgIGlmIChtaW51cy5MMk5vcm0oKSA8IE1hdGguU3FydChtaW4pKSByZXR1cm4gMC4wO1xyXG4gICAgICAgICAgICBrID0gayAvIGsuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHBwID0gcCAtIChwICogayAqIGspO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxcCA9IHEgLSAocSAqIGsgKiBrKTtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGVwcCA9IHBwIC8gcHAuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGVxcCA9IHFwIC8gcXAuTDJOb3JtKCk7XHJcblxyXG4gICAgICAgICAgICBkb3VibGUgdGhldGEgPSBTdWJwcm9ibGVtMChlcHAsIGVxcCwgayk7XHJcbiAgICAgICAgICAgIGlmIChNYXRoLkFicyhwLkwyTm9ybSgpIC0gcS5MMk5vcm0oKSkgPiAocC5MMk5vcm0oKSAqIChNYXRoLlBvdygxMCwtOCkpKSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJXQVJOSU5HOnx8cHx8IGFuZCB8fHF8fCBtdXN0IGJlIHRoZSBzYW1lISEhXCIpO1xyXG4gICAgICAgICAgICBcclxuICAgICAgICAgICAgcmV0dXJuIHRoZXRhO1xyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGRvdWJsZVtdIFN1YnByb2JsZW0yKFZlY3Rvcjxkb3VibGU+IHAsIFZlY3Rvcjxkb3VibGU+IHEsIFZlY3Rvcjxkb3VibGU+IGsxLCBWZWN0b3I8ZG91YmxlPiBrMilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGRvdWJsZSBtaW4gPSBkb3VibGUuRXBzaWxvbjtcclxuICAgICAgICAgICAgZG91YmxlIGsxMiA9IGsxLkRvdFByb2R1Y3QoazIpO1xyXG4gICAgICAgICAgICBkb3VibGUgcGsgPSBwLkRvdFByb2R1Y3QoazIpO1xyXG4gICAgICAgICAgICBkb3VibGUgcWsgPSBxLkRvdFByb2R1Y3QoazEpO1xyXG4gICAgICAgICAgICBpZiAoTWF0aC5BYnMoMSAtIE1hdGguUG93KGsxMiwgMikpIDwgbWluKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIldBUk5JTkc6Tm8gc29sdXRpb24gZm91bmQgazEgIT0gazJcIik7XHJcbiAgICAgICAgICAgICAgICByZXR1cm4gbmV3IGRvdWJsZVswXTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBhbWF0cml4ID0gbV9idWlsZGVyLkRlbnNlKDIsIDIpO1xyXG4gICAgICAgICAgICBhbWF0cml4WzAsIDBdID0gazEyO1xyXG4gICAgICAgICAgICBhbWF0cml4WzAsIDFdID0gLTE7XHJcbiAgICAgICAgICAgIGFtYXRyaXhbMSwgMF0gPSAtMTtcclxuICAgICAgICAgICAgYW1hdHJpeFsxLCAxXSA9IGsxMjtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gYXZlY3RvciA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyBwaywgcWsgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGEgPSBhbWF0cml4Lk11bHRpcGx5KGF2ZWN0b3IgLyAoTWF0aC5Qb3coazEyLCAyKSAtIDEpKTtcclxuICAgICAgICAgICAgZG91YmxlIGJiID0gcC5Eb3RQcm9kdWN0KHApIC0gYS5Eb3RQcm9kdWN0KGEpIC0gMiAqIGFbMF0gKiBhWzFdICogazEyO1xyXG4gICAgICAgICAgICBpZiAoTWF0aC5BYnMoYmIpIDwgbWluKSBiYiA9IDA7XHJcbiAgICAgICAgICAgIGlmIChiYiA8IDApXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiV0FSTklORzpObyBzb2x1dGlvbiBmb3VuZCBubyBpbnRlcnNlY3Rpb24gZm91bmQgYmV0d2VlbiBjb25lc1wiKTtcclxuICAgICAgICAgICAgICAgIHJldHVybiBuZXcgZG91YmxlWzBdO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGRvdWJsZSBnYW1tYSA9IE1hdGguU3FydChiYikgLyBDcm9zcyhrMSwgazIpLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICBpZiAoTWF0aC5BYnMoZ2FtbWEpIDwgbWluKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBjbWFsdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93VmVjdG9ycyhrMSwgazIsIENyb3NzKGsxLCBrMikpO1xyXG4gICAgICAgICAgICAgICAgY21hbHQgPSBjbWFsdC5UcmFuc3Bvc2UoKTtcclxuICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGMxdmVjYWx0ID0gdl9idWlsZGVyLkRlbnNlKDMpO1xyXG4gICAgICAgICAgICAgICAgYzF2ZWNhbHRbMF0gPSBhWzBdO1xyXG4gICAgICAgICAgICAgICAgYzF2ZWNhbHRbMV0gPSBhWzFdO1xyXG4gICAgICAgICAgICAgICAgYzF2ZWNhbHRbMl0gPSBnYW1tYTtcclxuXHJcbiAgICAgICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBjMWFsdCA9IGNtYWx0Lk11bHRpcGx5KGMxdmVjYWx0KTtcclxuICAgICAgICAgICAgICAgIGRvdWJsZSB0aGV0YTIgPSBTdWJwcm9ibGVtMShrMiwgcCwgYzFhbHQpO1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIHRoZXRhMSA9IC1TdWJwcm9ibGVtMShrMSwgcSwgYzFhbHQpO1xyXG4gICAgICAgICAgICAgICAgZG91YmxlW10gdGhldGFzZmlyc3QgPSB7IHRoZXRhMSwgdGhldGEyIH07XHJcbiAgICAgICAgICAgICAgIFxyXG4gICAgICAgICAgICAgICAgXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gdGhldGFzZmlyc3Q7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gY20gPSBtX2J1aWxkZXIuRGVuc2VPZlJvd1ZlY3RvcnMoazEsIGsyLCBDcm9zcyhrMSwgazIpKTtcclxuICAgICAgICAgICAgY20gPSBjbS5UcmFuc3Bvc2UoKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gYzF2ZWMgPSB2X2J1aWxkZXIuRGVuc2UoMyk7XHJcbiAgICAgICAgICAgIGMxdmVjWzBdID0gYVswXTtcclxuICAgICAgICAgICAgYzF2ZWNbMV0gPSBhWzFdO1xyXG4gICAgICAgICAgICBjMXZlY1syXSA9IGdhbW1hO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBjMnZlYyA9IHZfYnVpbGRlci5EZW5zZSgzKTtcclxuICAgICAgICAgICAgYzJ2ZWNbMF0gPSBhWzBdO1xyXG4gICAgICAgICAgICBjMnZlY1sxXSA9IGFbMV07XHJcbiAgICAgICAgICAgIGMydmVjWzJdID0gLWdhbW1hO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBjMSA9IGNtLk11bHRpcGx5KGMxdmVjKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gYzIgPSBjbS5NdWx0aXBseShjMnZlYyk7XHJcblxyXG4gICAgICAgICAgICBkb3VibGUgdGhldGExXzEgPSAtU3VicHJvYmxlbTEocSwgYzEsIGsxKTtcclxuICAgICAgICAgICAgZG91YmxlIHRoZXRhMV8yID0gLVN1YnByb2JsZW0xKHEsIGMyLCBrMSk7XHJcbiAgICAgICAgICAgIGRvdWJsZSB0aGV0YTJfMSA9IFN1YnByb2JsZW0xKHAsIGMxLCBrMik7XHJcbiAgICAgICAgICAgIGRvdWJsZSB0aGV0YTJfMiA9IFN1YnByb2JsZW0xKHAsIGMyLCBrMik7XHJcbiAgICAgICAgICAgIGRvdWJsZVtdIHRoZXRhcyA9IG5ldyBkb3VibGVbNF0geyB0aGV0YTFfMSwgdGhldGEyXzEsIHRoZXRhMV8yLCB0aGV0YTJfMiB9O1xyXG4gICAgICAgICAgICByZXR1cm4gdGhldGFzO1xyXG4gICAgICAgICAgICAvL05PVEU6VEhJUyBET0VTIE5PVCBSRVRVUk4gTVVMVElESU0gQVJSQVkgTElLRSBQWVRIT04gVkVSU0lPTlxyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGRvdWJsZVtdIFN1YnByb2JsZW0zKFZlY3Rvcjxkb3VibGU+IHAsIFZlY3Rvcjxkb3VibGU+IHEsIFZlY3Rvcjxkb3VibGU+IGssIGRvdWJsZSBkKVxyXG4gICAgICAgIHtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHBwID0gcCAtICgocCAqIGspICogayk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFwID0gcSAtICgocSAqIGspICogayk7XHJcbiAgICAgICAgICAgIGRvdWJsZSBkcHNxID0gTWF0aC5Qb3coZCwgMikgLSBNYXRoLlBvdygoay5Eb3RQcm9kdWN0KHAgKyBxKSksIDIpO1xyXG4gICAgICAgICAgICBkb3VibGUgYmIgPSAtKHBwLkRvdFByb2R1Y3QocHApICsgcXAuRG90UHJvZHVjdChxcCkgLSBkcHNxKSAvICgyICogcHAuTDJOb3JtKCkgKiBxcC5MMk5vcm0oKSk7XHJcbiAgICAgICAgICAgIGlmIChkcHNxIDwgMCB8fCBNYXRoLkFicyhiYikgPiAxKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIk5vIHNvbHV0aW9uIG5vIHJvdGF0aW9uIGNhbiBhY2hpZXZlIHNwZWNpZmllZCBkaXN0YW5jZVwiKTtcclxuICAgICAgICAgICAgICAgIHJldHVybiBuZXcgZG91YmxlWzBdO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGRvdWJsZSB0aGV0YSA9IFN1YnByb2JsZW0xKHBwIC8gcHAuTDJOb3JtKCksIHFwIC8gcXAuTDJOb3JtKCksIGspO1xyXG4gICAgICAgICAgICBkb3VibGUgcGhpID0gTWF0aC5BY29zKGJiKTtcclxuICAgICAgICAgICAgLy9Db25zb2xlLldyaXRlTGluZShcInRoZXRhLCBwaGkgezB9LCB7MX1cIiwgdGhldGEsIHBoaSk7XHJcbiAgICAgICAgICAgIGlmIChNYXRoLkFicyhwaGkpID4gMClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIG5ldyBkb3VibGVbMl0geyB0aGV0YSArIHBoaSwgdGhldGEgLSBwaGkgfTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiBuZXcgZG91YmxlWzFdIHsgdGhldGEgfTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBkb3VibGVbXSBTdWJwcm9ibGVtNChWZWN0b3I8ZG91YmxlPiBwLCBWZWN0b3I8ZG91YmxlPiBxLCBWZWN0b3I8ZG91YmxlPiBrLCBkb3VibGUgZClcclxuICAgICAgICB7XHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBoYXR0ZWQgPSBIYXQoayk7XHJcbiAgICAgICAgICAgIGRvdWJsZSBhID0gaGF0dGVkLkxlZnRNdWx0aXBseShwKS5Eb3RQcm9kdWN0KHEpO1xyXG5cclxuICAgICAgICAgICAgZG91YmxlIGIgPSAtMSpoYXR0ZWQuTGVmdE11bHRpcGx5KGhhdHRlZC5MZWZ0TXVsdGlwbHkocSkpLkRvdFByb2R1Y3QocCk7XHJcbiAgICAgICAgICAgIGRvdWJsZSBjID0gZCAtIChwLkRvdFByb2R1Y3QocSkgLSBiKTtcclxuICAgICAgICAgICAgZG91YmxlIHBoaSA9IE1hdGguQXRhbjIoYiwgYSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGRwcmVwID0gdl9idWlsZGVyLkRlbnNlKDIpO1xyXG4gICAgICAgICAgICBkcHJlcFswXSA9IGE7XHJcbiAgICAgICAgICAgIGRwcmVwWzFdID0gYjtcclxuICAgICAgICAgICAgZCA9IGMgLyBkcHJlcC5MMk5vcm0oKTtcclxuICAgICAgICAgICAgaWYoZCA+IDEpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiBuZXcgZG91YmxlWzBdO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGRvdWJsZSBwc2kgPSBNYXRoLkFzaW4oZCk7XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICByZXR1cm4gbmV3IGRvdWJsZVsyXSB7IC1waGkrcHNpLC0gcGhpLXBzaStNYXRoLlBJIH07XHJcblxyXG4gICAgICAgIH1cclxuXHJcbiAgICB9XHJcblxyXG5cclxuXHJcbiAgICBwdWJsaWMgY2xhc3MgUm9ib3RcclxuICAgIHtcclxuICAgICAgICBwdWJsaWMgUm9ib3QoKSB7IH1cclxuICAgICAgICBwdWJsaWMgTWF0cml4PGRvdWJsZT4gSCB7IGdldDsgc2V0OyB9XHJcbiAgICAgICAgcHVibGljIE1hdHJpeDxkb3VibGU+IFAgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHB1YmxpYyBJbnQzMltdIEpvaW50X3R5cGUgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHB1YmxpYyBkb3VibGVbXSBKb2ludF9sb3dlcl9saW1pdCB7IGdldDsgc2V0OyB9XHJcbiAgICAgICAgcHVibGljIGRvdWJsZVtdIEpvaW50X3VwcGVyX2xpbWl0IHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgZG91YmxlW10gSm9pbnRfdmVsX2xpbWl0IHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgZG91YmxlW10gSm9pbnRfYWNjX2xpbWl0IHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgTWF0cml4PGRvdWJsZT5bXSBNIHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgTWF0cml4PGRvdWJsZT4gUl90b29sIHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgVmVjdG9yPGRvdWJsZT4gUF90b29sIHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgc3RyaW5nW10gSm9pbnRfbmFtZXMgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHB1YmxpYyBzdHJpbmcgUm9vdF9saW5rX25hbWUgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHB1YmxpYyBzdHJpbmcgVGlwX2xpbmtfbmFtZSB7IGdldDsgc2V0OyB9XHJcblxyXG4gICAgICAgIHB1YmxpYyBSb2JvdChNYXRyaXg8ZG91YmxlPiBoLCBNYXRyaXg8ZG91YmxlPiBwLCBJbnQzMltdIGpvaW50X3R5cGUsIGRvdWJsZVtdIGpvaW50X2xvd2VyX2xpbWl0ID0gZGVmYXVsdChkb3VibGVbXSksIGRvdWJsZVtdIGpvaW50X3VwcGVyX2xpbWl0ID0gZGVmYXVsdChkb3VibGVbXSksIGRvdWJsZVtdIGpvaW50X3ZlbF9saW1pdCA9IGRlZmF1bHQoZG91YmxlW10pLCBkb3VibGVbXSBqb2ludF9hY2NfbGltaXQgPSBkZWZhdWx0KGRvdWJsZVtdKSwgTWF0cml4PGRvdWJsZT5bXSBtID0gZGVmYXVsdChNYXRyaXg8ZG91YmxlPltdKSxcclxuICAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByX3Rvb2wgPSBkZWZhdWx0KE1hdHJpeDxkb3VibGU+KSwgVmVjdG9yPGRvdWJsZT4gcF90b29sID0gZGVmYXVsdChWZWN0b3I8ZG91YmxlPiksIHN0cmluZ1tdIGpvaW50X25hbWVzID0gZGVmYXVsdChzdHJpbmdbXSksIHN0cmluZyByb290X2xpbmtfbmFtZSA9IGRlZmF1bHQoc3RyaW5nKSwgc3RyaW5nIHRpcF9saW5rX25hbWUgPSBkZWZhdWx0KHN0cmluZykpXHJcbiAgICAgICAge1xyXG5TeXN0ZW0uRnVuYzxkb3VibGUsIGRvdWJsZSwgYm9vbD4gQWxtb3N0RXF1YWxzID0gbnVsbDtcbiAgICAgICAgICAgIEludDMyW10gam9pbnRfdHlwZXMgPSBuZXcgSW50MzJbXSB7IDAsIDEsIDIsIDMgfTtcclxuICAgICAgICAgICAgSUVudW1lcmFibGU8VmVjdG9yPGRvdWJsZT4+IHNwbGljZXIgPSBoLkVudW1lcmF0ZUNvbHVtbnMoKTtcclxuICAgICAgICAgICAgXHJcbkFsbW9zdEVxdWFscyA9ICh2YWwxLCB2YWwyKSA9PlxyXG57XHJcbiAgICBpZiAoTWF0aC5BYnModmFsMSAtIHZhbDIpIDwgKE1hdGguUG93KDEwLCAtOCkpKVxyXG4gICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG59XHJcblxyXG47XG4gICAgICAgICAgICBmb3JlYWNoIChWZWN0b3I8ZG91YmxlPiBjb2x1bW4gaW4gc3BsaWNlcilcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgaWYgKCEoQWxtb3N0RXF1YWxzKGNvbHVtbi5MMk5vcm0oKSwxKSkpIHRocm93IG5ldyBBcmd1bWVudEV4Y2VwdGlvbihTdHJpbmcuRm9ybWF0KFwiTWF0cml4IEggaXMgbm90IEFjY2VwdGFibGVcIikpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgam9pbnRfdHlwZS5MZW5ndGg7IGkrKylcclxuICAgICAgICAgICAge1xyXG5cclxuICAgICAgICAgICAgICAgIGlmICghKFN5c3RlbS5BcnJheUV4dGVuc2lvbnMuQ29udGFpbnM8aW50Pihqb2ludF90eXBlcyxqb2ludF90eXBlW2ldKSkpIHRocm93IG5ldyBBcmd1bWVudEV4Y2VwdGlvbihTdHJpbmcuRm9ybWF0KFwiSm9pbnQgdHlwZXMgY29udGFpbnMgaW5jb3JyZWN0IHZhbHVlc1wiKSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgaWYgKGguUm93Q291bnQgIT0gMykgdGhyb3cgbmV3IEFyZ3VtZW50RXhjZXB0aW9uKFN0cmluZy5Gb3JtYXQoXCJNYXRyaXggSCBpcyBub3QgQWNjZXB0YWJsZVwiKSk7XHJcbiAgICAgICAgICAgIGlmIChwLlJvd0NvdW50ICE9IDMpIHRocm93IG5ldyBBcmd1bWVudEV4Y2VwdGlvbihTdHJpbmcuRm9ybWF0KFwiTWF0cml4IFAgaXMgbm90IEFjY2VwdGFibGVcIikpO1xyXG4gICAgICAgICAgICBpZiAoaC5Db2x1bW5Db3VudCArIDEgIT0gcC5Db2x1bW5Db3VudCB8fCBoLkNvbHVtbkNvdW50ICE9IGpvaW50X3R5cGUuTGVuZ3RoKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIk1hdHJpeCBEaW1lbnNpb25zIGFyZSBub3QgQWNjZXB0YWJsZVwiKSk7XHJcbiAgICAgICAgICAgIGlmIChqb2ludF9sb3dlcl9saW1pdCAhPSBudWxsICYmIGpvaW50X3VwcGVyX2xpbWl0ICE9IG51bGwpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChqb2ludF9sb3dlcl9saW1pdC5MZW5ndGggIT0gam9pbnRfdHlwZS5MZW5ndGggfHwgam9pbnRfdHlwZS5MZW5ndGggIT0gam9pbnRfdXBwZXJfbGltaXQuTGVuZ3RoKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIkpvaW50IExpbWl0cyBub3QgQWNjZXB0YWJsZVwiKSk7XHJcbiAgICAgICAgICAgICAgICBKb2ludF91cHBlcl9saW1pdCA9IGpvaW50X3VwcGVyX2xpbWl0O1xyXG4gICAgICAgICAgICAgICAgSm9pbnRfbG93ZXJfbGltaXQgPSBqb2ludF9sb3dlcl9saW1pdDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIEpvaW50X2xvd2VyX2xpbWl0ID0gbnVsbDtcclxuICAgICAgICAgICAgICAgIEpvaW50X3VwcGVyX2xpbWl0ID0gbnVsbDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBpZiAoam9pbnRfdmVsX2xpbWl0ICE9IG51bGwpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChqb2ludF92ZWxfbGltaXQuTGVuZ3RoICE9IGpvaW50X3R5cGUuTGVuZ3RoKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIkpvaW50IFZlbG9jaXRpZXMgbm90IEFjY2VwdGFibGVcIikpO1xyXG4gICAgICAgICAgICAgICAgSm9pbnRfdmVsX2xpbWl0ID0gam9pbnRfdmVsX2xpbWl0O1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgSm9pbnRfdmVsX2xpbWl0ID0gbnVsbDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBpZiAoam9pbnRfYWNjX2xpbWl0ICE9IG51bGwpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChqb2ludF9hY2NfbGltaXQuTGVuZ3RoICE9IGpvaW50X3R5cGUuTGVuZ3RoKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIkpvaW50IEFjY2VsZXJhdGlvbnMgbm90IEFjY2VwdGFibGVcIikpO1xyXG4gICAgICAgICAgICAgICAgSm9pbnRfYWNjX2xpbWl0ID0gam9pbnRfYWNjX2xpbWl0O1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgSm9pbnRfYWNjX2xpbWl0ID0gbnVsbDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBpZiAobSAhPSBudWxsKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBpZiAobS5MZW5ndGggIT0gSC5Db2x1bW5Db3VudCkgdGhyb3cgbmV3IEFyZ3VtZW50RXhjZXB0aW9uKFN0cmluZy5Gb3JtYXQoXCJJbmVydGlhIE1hdHJpY2VzIG5vdCBBY2NlcHRhYmxlXCIpKTtcclxuICAgICAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgbS5MZW5ndGg7IGkrKylcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBpZiAobVtpXS5Db2x1bW5Db3VudCAhPSA2IHx8IG1baV0uUm93Q291bnQgIT0gNikgdGhyb3cgbmV3IEFyZ3VtZW50RXhjZXB0aW9uKFN0cmluZy5Gb3JtYXQoXCJJbmVydGlhIE1hdHJpY2VzIG5vdCBBY2NlcHRhYmxlXCIpKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIE0gPSBtO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgTSA9IG51bGw7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgaWYgKHJfdG9vbCAhPSBudWxsICYmIHBfdG9vbCAhPSBudWxsKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBSX3Rvb2wgPSByX3Rvb2w7XHJcbiAgICAgICAgICAgICAgICBQX3Rvb2wgPSBwX3Rvb2w7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBSX3Rvb2wgPSBudWxsO1xyXG4gICAgICAgICAgICAgICAgUF90b29sID0gbnVsbDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBIID0gaDtcclxuICAgICAgICAgICAgUCA9IHA7XHJcbiAgICAgICAgICAgIEpvaW50X3R5cGUgPSBqb2ludF90eXBlO1xyXG4gICAgICAgICAgICBpZiAoam9pbnRfbmFtZXMgIT0gbnVsbClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgaWYgKGpvaW50X25hbWVzLkxlbmd0aCAhPSBqb2ludF90eXBlLkxlbmd0aCkgdGhyb3cgbmV3IEFyZ3VtZW50RXhjZXB0aW9uKFN0cmluZy5Gb3JtYXQoXCJKb2ludCBOYW1lcyBub3QgQWNjZXB0YWJsZVwiKSk7XHJcbiAgICAgICAgICAgICAgICBKb2ludF9uYW1lcyA9IGpvaW50X25hbWVzO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgSm9pbnRfbmFtZXMgPSBudWxsO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIFJvb3RfbGlua19uYW1lID0gcm9vdF9saW5rX25hbWU7XHJcbiAgICAgICAgICAgIFRpcF9saW5rX25hbWUgPSB0aXBfbGlua19uYW1lO1xyXG5cclxuXHJcbiAgICAgICAgfVxyXG5cclxuICAgIH1cclxuICAgIHB1YmxpYyBjbGFzcyBUcmFuc2Zvcm1cclxuICAgIHtcclxuICAgICAgICBwdWJsaWMgVHJhbnNmb3JtKCkgeyB9XHJcbiAgICAgICAgcHVibGljIE1hdHJpeDxkb3VibGU+IFIgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHB1YmxpYyBWZWN0b3I8ZG91YmxlPiBQIHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgc3RyaW5nIFBhcmVudF9mcmFtZV9pZCB7IGdldDsgc2V0OyB9XHJcbiAgICAgICAgcHVibGljIHN0cmluZyBDaGlsZF9mcmFtZV9pZCB7IGdldDsgc2V0OyB9XHJcbiAgICAgICAgXHJcbiAgICAgICAgcHVibGljIFRyYW5zZm9ybShNYXRyaXg8ZG91YmxlPiByLCBWZWN0b3I8ZG91YmxlPiBwLCBzdHJpbmcgcGFyZW50X2ZyYW1lX2lkID0gZGVmYXVsdChzdHJpbmcpLCBzdHJpbmcgY2hpbGRfZnJhbWVfaWQgPSBkZWZhdWx0KHN0cmluZykpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAoci5Db2x1bW5Db3VudCAhPSAzIHx8IHIuUm93Q291bnQgIT0gMykgdGhyb3cgbmV3IEFyZ3VtZW50RXhjZXB0aW9uKFN0cmluZy5Gb3JtYXQoXCJSb3RhdGlvbiBNYXRyaXggZm9yIHRyYW5zZm9ybSBub3QgQWNjZXB0YWJsZVwiKSk7XHJcbiAgICAgICAgICAgIGlmKHAuQ291bnQhPTMpIHRocm93IG5ldyBBcmd1bWVudEV4Y2VwdGlvbihTdHJpbmcuRm9ybWF0KFwiUG9zaXRpb24gVmVjdG9yIGZvciB0cmFuc2Zvcm0gbm90IEFjY2VwdGFibGVcIikpO1xyXG4gICAgICAgICAgICBSID0gcjtcclxuICAgICAgICAgICAgUCA9IHA7XHJcbiAgICAgICAgICAgIFBhcmVudF9mcmFtZV9pZCA9IHBhcmVudF9mcmFtZV9pZDtcclxuICAgICAgICAgICAgQ2hpbGRfZnJhbWVfaWQgPSBjaGlsZF9mcmFtZV9pZDtcclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIHN0YXRpYyBUcmFuc2Zvcm0gb3BlcmF0b3IgKihUcmFuc2Zvcm0gdHJhbjEsIFRyYW5zZm9ybSB0cmFuMilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIgPSB0cmFuMS5SKiB0cmFuMi5SO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBQID0gdHJhbjEuUCArIHRyYW4xLlIgKiB0cmFuMi5QO1xyXG4gICAgICAgICAgICBUcmFuc2Zvcm0gb3V0cHV0PSBuZXcgVHJhbnNmb3JtKFIsIFAsIHRyYW4xLlBhcmVudF9mcmFtZV9pZCwgdHJhbjIuQ2hpbGRfZnJhbWVfaWQpO1xyXG4gICAgICAgICAgICByZXR1cm4gb3V0cHV0O1xyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGJvb2wgb3BlcmF0b3IgPT0oVHJhbnNmb3JtIHRyYW4xLCBUcmFuc2Zvcm0gdHJhbjIpXHJcbiAgICAgICAge1xyXG5TeXN0ZW0uRnVuYzxkb3VibGUsIGRvdWJsZSwgYm9vbD4gQWxtb3N0RXF1YWxzID0gbnVsbDtcbiAgICAgICAgICAgIFxyXG5BbG1vc3RFcXVhbHMgPSAodmFsMSwgdmFsMikgPT5cclxue1xyXG4gICAgaWYgKE1hdGguQWJzKHZhbDEgLSB2YWwyKSA8IChNYXRoLlBvdygxMCwgLTgpKSlcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIHJldHVybiBmYWxzZTtcclxufVxyXG5cclxuO1xuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCAzOyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGZvciAoaW50IGogPSAwOyBpIDwgMzsgaSsrKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGlmICghKEFsbW9zdEVxdWFscyh0cmFuMS5SW2ksIGpdLHRyYW4yLlJbaSwgal0pKSkgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgMzsgaSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBpZiAoIShBbG1vc3RFcXVhbHModHJhbjEuUFtpXSwgdHJhbjIuUFtpXSkpKSByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgcmV0dXJuIHRydWU7XHJcblxyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGJvb2wgb3BlcmF0b3IgIT0oVHJhbnNmb3JtIHRyYW4xLCBUcmFuc2Zvcm0gdHJhbjIpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gKCEodHJhbjEgPT0gdHJhbjIpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIFRyYW5zZm9ybSBpbnYoVHJhbnNmb3JtIHRyYW4xKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUiA9IHRyYW4xLlIuVHJhbnNwb3NlKCk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IFAgPSAtKHRyYW4xLlIqdHJhbjEuUCk7XHJcbiAgICAgICAgICAgIFRyYW5zZm9ybSBvdXRwdXQgPSBuZXcgVHJhbnNmb3JtKFIsIFAsIHRyYW4xLlBhcmVudF9mcmFtZV9pZCwgdHJhbjEuQ2hpbGRfZnJhbWVfaWQpO1xyXG4gICAgICAgICAgICByZXR1cm4gb3V0cHV0O1xyXG4gICAgICAgIH1cclxuICAgIH1cclxufVxyXG4iLCJ1c2luZyBTeXN0ZW07XHJcbnVzaW5nIFN5c3RlbS5Db2xsZWN0aW9ucy5HZW5lcmljO1xyXG51c2luZyBTeXN0ZW0uTGlucTtcclxudXNpbmcgU3lzdGVtLlRleHQ7XHJcbnVzaW5nIFN5c3RlbS5UaHJlYWRpbmcuVGFza3M7XHJcbnVzaW5nIE1hdGhOZXQuTnVtZXJpY3M7XHJcbnVzaW5nIE1hdGhOZXQuTnVtZXJpY3MuTGluZWFyQWxnZWJyYTtcclxudXNpbmcgTWF0aE5ldC5OdW1lcmljcy5MaW5lYXJBbGdlYnJhLkRvdWJsZS5NYXRoTmV0Lk51bWVyaWNzLkxpbmVhckFsZ2VicmE7XHJcblxyXG5uYW1lc3BhY2UgVGVzdEdlbmVyYWxSb2JvdGljc1Rvb2xib3hORVRcclxue1xyXG4gICAgcHVibGljIGNsYXNzIE5vcm1hbGl6ZUpvaW50c1xyXG4gICAge1xyXG4gICAgICAgIHN0YXRpYyBWZWN0b3JCdWlsZGVyPGRvdWJsZT4gdl9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uVmVjdG9yO1xyXG4gICAgICAgIHN0YXRpYyBNYXRyaXhCdWlsZGVyPGRvdWJsZT4gbV9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uTWF0cml4O1xyXG4gICAgICAgIHB1YmxpYyBOb3JtYWxpemVKb2ludHMoKSB7IH1cclxuXHJcbiAgICAgICAgcHVibGljIFJvYm90IFJvYm90IHsgZ2V0OyBzZXQ7IH1cclxuICAgICAgICBwdWJsaWMgZG91YmxlW10gTGFzdF9qb2ludHMgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHByaXZhdGUgYm9vbCBjaGVja19saW1pdHMgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHByaXZhdGUgYm9vbCB1c2VfbGFzdF9qb2ludHMgeyBnZXQ7IHNldDsgfVxyXG4gICAgICAgIHByaXZhdGUgZG91YmxlIGN1cnJlbnRfY29tcGFyZSB7IGdldDsgc2V0OyB9XHJcbiAgICAgICAgcHJpdmF0ZSBkb3VibGVbXSBjdXJyZW50X2NvbXBhcmUyIHsgZ2V0OyBzZXQ7IH1cclxuXHJcblxyXG4gICAgICAgIHB1YmxpYyBOb3JtYWxpemVKb2ludHMoUm9ib3Qgcm9ib3QsIGRvdWJsZVtdIGxhc3Rfam9pbnRzID0gZGVmYXVsdChkb3VibGVbXSkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBSb2JvdCA9IHJvYm90O1xyXG4gICAgICAgICAgICBMYXN0X2pvaW50cyA9IGxhc3Rfam9pbnRzO1xyXG4gICAgICAgICAgICBjaGVja19saW1pdHMgPSBSb2JvdC5Kb2ludF91cHBlcl9saW1pdCAhPSBudWxsICYmIFJvYm90LkpvaW50X2xvd2VyX2xpbWl0ICE9IG51bGw7XHJcbiAgICAgICAgICAgIHVzZV9sYXN0X2pvaW50cyA9IGxhc3Rfam9pbnRzICE9IGRlZmF1bHQoZG91YmxlW10pO1xyXG4gICAgICAgICAgICAvL2lmIChyLkNvbHVtbkNvdW50ICE9IDMgfHwgci5Sb3dDb3VudCAhPSAzKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIlJvdGF0aW9uIE1hdHJpeCBmb3IgdHJhbnNmb3JtIG5vdCBBY2NlcHRhYmxlXCIpKTtcclxuICAgICAgICAgICAgLy9pZiAocC5Db3VudCAhPSAzKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIlBvc2l0aW9uIFZlY3RvciBmb3IgdHJhbnNmb3JtIG5vdCBBY2NlcHRhYmxlXCIpKTtcclxuICAgICAgICAgICAgLy9SID0gcjtcclxuICAgICAgICAgICAgLy9QID0gcDtcclxuICAgICAgICAgICAgLy9QYXJlbnRfZnJhbWVfaWQgPSBwYXJlbnRfZnJhbWVfaWQ7XHJcbiAgICAgICAgICAgIC8vQ2hpbGRfZnJhbWVfaWQgPSBjaGlsZF9mcmFtZV9pZDtcclxuICAgICAgICB9XHJcbiAgICAgICAgcHVibGljIGRvdWJsZSBOb3JtYWxpemUoaW50IGpvaW50X2luZGV4LCBkb3VibGUgdGhldGEpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBkb3VibGUgdSA9IFJvYm90LkpvaW50X3VwcGVyX2xpbWl0W2pvaW50X2luZGV4XTtcclxuICAgICAgICAgICAgZG91YmxlIGwgPSBSb2JvdC5Kb2ludF9sb3dlcl9saW1pdFtqb2ludF9pbmRleF07XHJcbiAgICAgICAgICAgIGlmIChjaGVja19saW1pdHMpXHJcbiAgICAgICAgICAgIHtcclxuXHJcbiAgICAgICAgICAgICAgICBpZiAoIShsIDwgdGhldGEgJiYgdGhldGEgPCB1KSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBhID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IC0xLjAsIDEuMCB9KTtcclxuICAgICAgICAgICAgICAgICAgICBhID0gYSAqIDIgKiBNYXRoLlBJO1xyXG4gICAgICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGIgPSBhLkFkZCh0aGV0YSk7XHJcbiAgICAgICAgICAgICAgICAgICAgaW50IGluZGV4ID0gLTE7XHJcbiAgICAgICAgICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCBiLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAobCA8IGJbaV0gJiYgYltpXSA8IHUpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGluZGV4ID0gaTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgIGlmIChpbmRleCA9PSAtMSlcclxuICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHJldHVybiBkZWZhdWx0KGRvdWJsZSk7XHJcbiAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhID0gdGhldGEgKyBhW2luZGV4XTtcclxuICAgICAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgaWYgKHVzZV9sYXN0X2pvaW50cylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIGRpZmYgPSBMYXN0X2pvaW50c1tqb2ludF9pbmRleF0gLSB0aGV0YTtcclxuXHJcbiAgICAgICAgICAgICAgICBpbnQgbl9kaWZmID0gKGludClNYXRoLkZsb29yKGRpZmYgLyAoTWF0aC5QSSAqIDIuMCkpO1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIHJfZGlmZiA9IGRpZmYgJSAoTWF0aC5QSSAqIDIuMCk7XHJcbiAgICAgICAgICAgICAgICBpZiAocl9kaWZmID4gTWF0aC5QSSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBuX2RpZmYgKz0gMTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIGlmIChNYXRoLkFicyhuX2RpZmYpID4gMClcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBpZiAoIWNoZWNrX2xpbWl0cylcclxuICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhICs9IDIgKiBNYXRoLlBJICogbl9kaWZmO1xyXG4gICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBkb3VibGUgdGhldGFfdnMgPSB0aGV0YSArIDIgKiBNYXRoLlBJO1xyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gdGhldGFfdiA9IHZfYnVpbGRlci5EZW5zZShNYXRoLkFicyhuX2RpZmYpKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCB0aGV0YV92LkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGlmIChuX2RpZmYgPiAwKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhX3ZbaV0gPSBuX2RpZmYgLSBpO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgZWxzZSBpZiAobl9kaWZmIDwgMCkge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhX3ZbaV0gPSBuX2RpZmYgKyBpO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhX3YgPSB0aGV0YV92ICogMiAqIE1hdGguUEk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhX3YgPSB0aGV0YV92LkFkZCh0aGV0YSk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIGludCBpbmRleCA9IC0xO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBmb3IgKGludCBpID0gMDsgaSA8IHRoZXRhX3YuQ291bnQ7IGkrKylcclxuICAgICAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgaWYgKGwgPCB0aGV0YV92W2ldICYmIHRoZXRhX3ZbaV0gPCB1KVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGluZGV4ID0gaTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBicmVhaztcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAoaW5kZXggIT0gLTEpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoZXRhID0gdGhldGFfdltpbmRleF07XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICB9XHJcblxyXG5cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm4gdGhldGE7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHB1YmxpYyBkb3VibGVbXSBGaW5kTm9ybWFsaXplZEpvaW50cyhpbnQgam9pbnRfaW5kZXgsIGRvdWJsZVtdIHRoZXRhcylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIExpc3Q8ZG91YmxlPiB0aGV0YV9ub3JtZWQgPSBuZXcgTGlzdDxkb3VibGU+KCk7XHJcbiAgICAgICAgICAgIGZvcmVhY2ggKGRvdWJsZSB0MSBpbiB0aGV0YXMpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGRvdWJsZSB0MyA9IE5vcm1hbGl6ZShqb2ludF9pbmRleCwgdDEpO1xyXG4gICAgICAgICAgICAgICAgaWYgKHQzICE9IGRlZmF1bHQoZG91YmxlKSkgdGhldGFfbm9ybWVkLkFkZCh0Myk7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIGlmICghdXNlX2xhc3Rfam9pbnRzIHx8IHRoZXRhX25vcm1lZC5Db3VudCA8IDIpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGRvdWJsZVtdIG91dHB1dCA9IG5ldyBkb3VibGVbdGhldGFfbm9ybWVkLkNvdW50XTtcclxuICAgICAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgdGhldGFfbm9ybWVkLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgb3V0cHV0W2ldID0gdGhldGFfbm9ybWVkW2ldO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIG91dHB1dDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAvLyB0aGV0YV9sYXN0ID0gdl9idWlsZGVyLkRlbnNlKGpvaW50X2luZGV4Lkxlbmd0aCk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHRoZXRhX25vcm1lZF92ZWMgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KHRoZXRhX25vcm1lZC5Ub0FycmF5KCkpO1xyXG5cclxuICAgICAgICAgICAgZG91YmxlIHRoZXRhX2xhc3QgPSBMYXN0X2pvaW50c1tqb2ludF9pbmRleF07XHJcblxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB0aGV0YV9kaXN0X3ZlYyA9IHRoZXRhX25vcm1lZF92ZWMuU3VidHJhY3QodGhldGFfbGFzdCk7XHJcbiAgICAgICAgICAgIHRoZXRhX2Rpc3RfdmVjID0gdGhldGFfZGlzdF92ZWMuUG9pbnR3aXNlQWJzKCk7XHJcbiAgICAgICAgICAgIC8vZG91YmxlW10gdGhldGFfZGlzdCA9IHRoZXRhX2Rpc3RfdmVjLlRvQXJyYXkoKTtcclxuICAgICAgICAgICAgZG91YmxlW10gdGhldGFfZGlzdCA9IHRoZXRhX2Rpc3RfdmVjLlRvQXJyYXkoKTtcclxuICAgICAgICAgICAgTGlzdDxkb3VibGU+IHRoZXRhX3JldDEgPSBuZXcgTGlzdDxkb3VibGU+KCk7XHJcbiAgICAgICAgICAgIGZvcmVhY2ggKGRvdWJsZSB0aGV0YSBpbiB0aGV0YV9ub3JtZWQpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGN1cnJlbnRfY29tcGFyZSA9IHRoZXRhO1xyXG4gICAgICAgICAgICAgICAgaWYgKE1hdGguQWJzKGN1cnJlbnRfY29tcGFyZSAtIHRoZXRhX2xhc3QpIDwgTWF0aC5QSSAvIDIuMCkgdGhldGFfcmV0MS5BZGQodGhldGEpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGlmICh0aGV0YV9yZXQxLkNvdW50ID09IDEpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiB0aGV0YV9yZXQxLlRvQXJyYXkoKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBpZiAodGhldGFfcmV0MS5Db3VudCA9PSAwKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB0aGV0YV9yZXQxID0gdGhldGFfbm9ybWVkO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGRvdWJsZVtdIHJldHVybmVkID0gbmV3IGRvdWJsZVt0aGV0YV9kaXN0Lkxlbmd0aF07XHJcbiAgICAgICAgICAgIGludCBpbmRleCA9IDA7XHJcbiAgICAgICAgICAgIExpc3Q8ZG91YmxlPiB0aGV0YV9kaXN0X2xpc3QgPSBTeXN0ZW0uTGlucS5FbnVtZXJhYmxlLlRvTGlzdDxkb3VibGU+KHRoZXRhX2Rpc3QpO1xyXG4gICAgICAgICAgICBmb3IgKGludCB6ID0gMDsgeiA8IHRoZXRhX2Rpc3QuTGVuZ3RoOyB6KyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGluZGV4ID0gdGhldGFfZGlzdF9saXN0LkluZGV4T2YodGhldGFfZGlzdF9saXN0Lk1pbigpKTtcclxuICAgICAgICAgICAgICAgIHJldHVybmVkW3pdID0gdGhldGFfbm9ybWVkW2luZGV4XTtcclxuICAgICAgICAgICAgICAgIHRoZXRhX2Rpc3RfbGlzdC5SZW1vdmVBdChpbmRleCk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgcmV0dXJuIHJldHVybmVkO1xyXG5cclxuXHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBwdWJsaWMgZG91YmxlW11bXSBGaW5kTm9ybWFsaXplZEpvaW50cyhpbnRbXSBqb2ludF9pbmRleCwgZG91YmxlW10gdGhldGFzKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgTGlzdDxMaXN0PGRvdWJsZT4+IHRoZXRhX25vcm1lZCA9IG5ldyBMaXN0PExpc3Q8ZG91YmxlPj4oKTtcclxuXHJcbiAgICAgICAgICAgIGZvcmVhY2ggKGRvdWJsZSB0MSBpbiB0aGV0YXMpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIExpc3Q8ZG91YmxlPiB0aGV0YV9ub3JtZWRfaW50ZXJtZWQgPSBuZXcgTGlzdDxkb3VibGU+KCk7XHJcbiAgICAgICAgICAgICAgICBmb3IgKGludCBpID0gMDsgaSA8IGpvaW50X2luZGV4Lkxlbmd0aDsgaSsrKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGRvdWJsZSB0NCA9IE5vcm1hbGl6ZShqb2ludF9pbmRleFtpXSwgdDEpO1xyXG4gICAgICAgICAgICAgICAgICAgIGlmICh0NCAhPSBkZWZhdWx0KGRvdWJsZSkpIHRoZXRhX25vcm1lZF9pbnRlcm1lZC5BZGQodDQpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgdGhldGFfbm9ybWVkLkFkZCh0aGV0YV9ub3JtZWRfaW50ZXJtZWQpO1xyXG4gICAgICAgICAgICB9XHJcblxyXG5cclxuXHJcbiAgICAgICAgICAgIGlmICghdXNlX2xhc3Rfam9pbnRzIHx8IHRoZXRhX25vcm1lZC5Db3VudCA8IDIpXHJcbiAgICAgICAgICAgIHtcclxuXHJcbiAgICAgICAgICAgICAgICBkb3VibGVbXVtdIG91dHB1dCA9IFN5c3RlbS5MaW5xLkVudW1lcmFibGUuU2VsZWN0PExpc3Q8ZG91YmxlPixkb3VibGVbXT4odGhldGFfbm9ybWVkLChGdW5jPExpc3Q8ZG91YmxlPixkb3VibGVbXT4pKGEgPT4gYS5Ub0FycmF5KCkpKS5Ub0FycmF5KCk7XHJcbiAgICAgICAgICAgICAgICByZXR1cm4gb3V0cHV0O1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHRoZXRhX2xhc3QgPSB2X2J1aWxkZXIuRGVuc2Uoam9pbnRfaW5kZXguTGVuZ3RoKTtcclxuXHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgam9pbnRfaW5kZXguTGVuZ3RoOyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuXHJcbiAgICAgICAgICAgICAgICB0aGV0YV9sYXN0W2ldID0gTGFzdF9qb2ludHNbam9pbnRfaW5kZXhbaV1dO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICBkb3VibGVbXSB0aGV0YV9kaXN0X2FyciA9IG5ldyBkb3VibGVbam9pbnRfaW5kZXguTGVuZ3RoXTtcclxuXHJcbiAgICAgICAgICAgIExpc3Q8TGlzdDxkb3VibGU+PiB0aGV0YV9yZXQgPSBuZXcgTGlzdDxMaXN0PGRvdWJsZT4+KCk7XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgam9pbnRfaW5kZXguTGVuZ3RoOyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuXHJcbiAgICAgICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB0aGV0YV9ub3JtZWRfdmVjID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheSh0aGV0YV9ub3JtZWRbaV0uVG9BcnJheSgpKTtcclxuICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHRoZXRhX2Rpc3RfdmVjID0gdGhldGFfbm9ybWVkX3ZlYy5TdWJ0cmFjdCh0aGV0YV9sYXN0W2ldKTtcclxuICAgICAgICAgICAgICAgIHRoZXRhX2Rpc3RfYXJyW2ldID0gdGhldGFfZGlzdF92ZWMuTDJOb3JtKCk7XHJcblxyXG4gICAgICAgICAgICAgICAgYm9vbCBwYXNzZWQgPSB0cnVlO1xyXG4gICAgICAgICAgICAgICAgZm9yZWFjaCAoZG91YmxlIHQgaW4gdGhldGFfZGlzdF92ZWMpXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgaWYgKCEoTWF0aC5BYnModCkgPCBNYXRoLlBJIC8gMi4wKSkgcGFzc2VkID0gZmFsc2U7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBpZiAocGFzc2VkKSB0aGV0YV9yZXQuQWRkKHRoZXRhX25vcm1lZFtpXSk7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIC8qaWYgKGpvaW50X2luZGV4Lkxlbmd0aD09IDEpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIFxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAgeyovXHJcblxyXG4gICAgICAgICAgICAvL1ZlY3Rvcjxkb3VibGU+IHRoZXRhX2Rpc3QgPSB0aGV0YV9kaXN0X3ZlYy5Qb2ludHdpc2VBYnMoKTtcclxuICAgICAgICAgICAgLy9qdXN0IG5lZWQgYXJyYXkgb2YgcG9pbnR3aXNlIG1hZ25pdHVkZSB2YWx1ZXMgXHJcbiAgICAgICAgICAgIC8vIGRvdWJsZVtdIHRoZXRhX2Rpc3QgPSB0aGV0YV9kaXN0X3ZlYy5Ub0FycmF5KCk7XHJcbiAgICAgICAgICAgIC8vfVxyXG5cclxuXHJcbiAgICAgICAgICAgIC8qZm9yZWFjaChkb3VibGUgdGhldGEgaW4gdGhldGFfbm9ybWVkKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBjdXJyZW50X2NvbXBhcmUgPSB0aGV0YTsgXHJcbiAgICAgICAgICAgICAgICBpZiAodGhldGFfbGFzdC5Gb3JBbGwodGhldGFfdmVjX3Rlc3QpKSB0aGV0YV9yZXQxLkFkZCh0aGV0YSk7XHJcbiAgICAgICAgICAgIH0qL1xyXG4gICAgICAgICAgICBpZiAodGhldGFfcmV0LkNvdW50ID09IDEpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiBTeXN0ZW0uTGlucS5FbnVtZXJhYmxlLlNlbGVjdDxMaXN0PGRvdWJsZT4sZG91YmxlW10+KHRoZXRhX3JldCwoRnVuYzxMaXN0PGRvdWJsZT4sZG91YmxlW10+KShhID0+IGEuVG9BcnJheSgpKSkuVG9BcnJheSgpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGlmICh0aGV0YV9yZXQuQ291bnQgPT0gMClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhldGFfcmV0ID0gdGhldGFfbm9ybWVkO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIC8vZG91YmxlW10gcmV0dXJuZWQgPSBuZXcgZG91YmxlW3RoZXRhX2Rpc3QuQ291bnRdO1xyXG4gICAgICAgICAgICBpbnQgaW5kZXggPSAwO1xyXG4gICAgICAgICAgICBMaXN0PGRvdWJsZT4gdGhldGFfZGlzdF9saXN0ID0gU3lzdGVtLkxpbnEuRW51bWVyYWJsZS5Ub0xpc3Q8ZG91YmxlPih0aGV0YV9kaXN0X2Fycik7XHJcbiAgICAgICAgICAgIExpc3Q8TGlzdDxkb3VibGU+PiByZXR1cm5lZCA9IG5ldyBMaXN0PExpc3Q8ZG91YmxlPj4oKTtcclxuICAgICAgICAgICAgZm9yIChpbnQgeiA9IDA7IHogPCB0aGV0YV9kaXN0X2xpc3QuQ291bnQ7IHorKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgaW5kZXggPSB0aGV0YV9kaXN0X2xpc3QuSW5kZXhPZih0aGV0YV9kaXN0X2xpc3QuTWluKCkpO1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuZWRbel0gPSB0aGV0YV9ub3JtZWRbaW5kZXhdO1xyXG4gICAgICAgICAgICAgICAgdGhldGFfZGlzdF9saXN0LlJlbW92ZUF0KGluZGV4KTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm4gU3lzdGVtLkxpbnEuRW51bWVyYWJsZS5TZWxlY3Q8TGlzdDxkb3VibGU+LGRvdWJsZVtdPihyZXR1cm5lZCwoRnVuYzxMaXN0PGRvdWJsZT4sZG91YmxlW10+KShhID0+IGEuVG9BcnJheSgpKSkuVG9BcnJheSgpO1xyXG5cclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBwdWJsaWMgY2xhc3MgSW52ZXJzZUtpblxyXG4gICAge1xyXG4gICAgICAgIHN0YXRpYyBWZWN0b3JCdWlsZGVyPGRvdWJsZT4gdl9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uVmVjdG9yO1xyXG4gICAgICAgIHN0YXRpYyBNYXRyaXhCdWlsZGVyPGRvdWJsZT4gbV9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uTWF0cml4O1xyXG5cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGRvdWJsZVtdW10gcm9ib3Q2X3NwaGVyaWNhbHdyaXN0X2ludmtpbihSb2JvdCByb2JvdCwgVHJhbnNmb3JtIGRlc2lyZWRfcG9zZSwgZG91YmxlW10gbGFzdF9qb2ludHMgPSBkZWZhdWx0KGRvdWJsZVtdKSlcclxuICAgICAgICB7XHJcblxyXG4gICAgICAgICAgICBpZiAocm9ib3QuUl90b29sICE9IGRlZmF1bHQoTWF0cml4PGRvdWJsZT4pICYmIHJvYm90LlBfdG9vbCAhPSBkZWZhdWx0KFZlY3Rvcjxkb3VibGU+KSl7XHJcbiAgICAgICAgICAgICAgICBcclxuICAgICAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHRyYW5zcG9zZWQ9IHJvYm90LlJfdG9vbC5UcmFuc3Bvc2UoKTtcclxuICAgICAgICAgICAgICAgIGRlc2lyZWRfcG9zZS5SID0gZGVzaXJlZF9wb3NlLlIuTXVsdGlwbHkodHJhbnNwb3NlZCk7XHJcbiAgICAgICAgICAgICAgICBkZXNpcmVkX3Bvc2UuUCA9IGRlc2lyZWRfcG9zZS5QLWRlc2lyZWRfcG9zZS5SKnJvYm90LlBfdG9vbDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBIID0gcm9ib3QuSC5DbG9uZSgpO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBQPSByb2JvdC5QLkNsb25lKCk7XHJcbiAgICAgICAgICAgIExpc3Q8ZG91YmxlW10+IHRoZXRhX3YgPSBuZXcgTGlzdDxkb3VibGVbXT4oKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gemVyb3MgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDAuMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gZXggPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAwLjAsIDAuMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gZXkgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAxLjAsIDAuMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gZXogPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLjAsIDEuMCB9KTtcclxuICAgICAgICAgICAgaWYgKCFQLkNvbHVtbig0KS5FcXVhbHMoemVyb3MpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBkb3VibGUgUDRfZCA9IFAuQ29sdW1uKDQpICogSC5Db2x1bW4oMyk7XHJcbiAgICAgICAgICAgICAgICBpZiAoIShQLkNvbHVtbig0KS5TdWJ0cmFjdChQNF9kICogSC5Db2x1bW4oMykpLkVxdWFscyh6ZXJvcykpICl0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIlJvYm90IG1heSBub3QgaGF2ZSBzcGhlcmljYWwgd3Jpc3RcIikpO1xyXG4gICAgICAgICAgICAgICAgUC5TZXRDb2x1bW4oMywgUC5Db2x1bW4oMykgKyBQLkNvbHVtbig0KSk7XHJcbiAgICAgICAgICAgICAgICBQLlNldENvbHVtbig0LCB6ZXJvcyk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgaWYgKCFQLkNvbHVtbig1KS5FcXVhbHMoemVyb3MpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBkb3VibGUgUDVfZCA9IFAuQ29sdW1uKDUpICogSC5Db2x1bW4oNSk7XHJcbiAgICAgICAgICAgICAgICBpZiAoIShQLkNvbHVtbig1KS5TdWJ0cmFjdChQNV9kICogSC5Db2x1bW4oNSkpLkVxdWFscyh6ZXJvcykpKSB0aHJvdyBuZXcgQXJndW1lbnRFeGNlcHRpb24oU3RyaW5nLkZvcm1hdChcIlJvYm90IG1heSBub3QgaGF2ZSBzcGhlcmljYWwgd3Jpc3RcIikpO1xyXG4gICAgICAgICAgICAgICAgUC5TZXRDb2x1bW4oNiwgUC5Db2x1bW4oNikgKyBQLkNvbHVtbig1KSk7XHJcbiAgICAgICAgICAgICAgICBQLlNldENvbHVtbig1LCB6ZXJvcyk7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIGRvdWJsZSBkMSA9IGV5KihQLkNvbHVtbigxKS5BZGQoUC5Db2x1bW4oMikpLkFkZChQLkNvbHVtbigzKSkpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB2MSA9IGRlc2lyZWRfcG9zZS5QLlN1YnRyYWN0KCBkZXNpcmVkX3Bvc2UuUiAqIFAuQ29sdW1uKDYpKTtcclxuICAgICAgICAgICAgZG91YmxlW10gUTEgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlN1YnByb2JsZW00KGV5LCB2MSwgLUguQ29sdW1uKDApLCBkMSk7XHJcbiAgICAgICAgICAgIE5vcm1hbGl6ZUpvaW50cyBub3JtYWxpemUgPSBuZXcgTm9ybWFsaXplSm9pbnRzKHJvYm90LCBsYXN0X2pvaW50cyk7XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICBkb3VibGVbXSBmaXJzdF9ub3JtYWxpemUgPSBub3JtYWxpemUuRmluZE5vcm1hbGl6ZWRKb2ludHMoMCwgUTEpO1xyXG4gICAgICAgICAgICBmb3JlYWNoIChkb3VibGUgcTEgaW4gZmlyc3Rfbm9ybWFsaXplKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBSMDEgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChILkNvbHVtbigwKSwgcTEpO1xyXG4gICAgICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcDI2X3ExID0gUjAxLlRyYW5zcG9zZVRoaXNBbmRNdWx0aXBseShkZXNpcmVkX3Bvc2UuUCAtIGRlc2lyZWRfcG9zZS5SICogUC5Db2x1bW4oNikpLlN1YnRyYWN0KFAuQ29sdW1uKDApLkFkZChQLkNvbHVtbigxKSkpO1xyXG4gICAgICAgICAgICAgICAgZG91YmxlIGQzID0gcDI2X3ExLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gdjMgPSBQLkNvbHVtbigyKTtcclxuICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHAzID0gUC5Db2x1bW4oMyk7XHJcbiAgICAgICAgICAgICAgICBkb3VibGVbXSBRMyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTMocDMsIHYzLCBILkNvbHVtbigyKSwgZDMpO1xyXG4gICAgICAgICAgICAgICAgZG91YmxlW10gc2Vjb25kX25vcm1hbGl6ZSA9IG5vcm1hbGl6ZS5GaW5kTm9ybWFsaXplZEpvaW50cygyLCBRMyk7XHJcbiAgICAgICAgICAgICAgICBmb3JlYWNoIChkb3VibGUgcTMgaW4gc2Vjb25kX25vcm1hbGl6ZSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBSMjMgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChILkNvbHVtbigyKSwgcTMpO1xyXG4gICAgICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHYyID0gcDI2X3ExO1xyXG4gICAgICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHAyID0gUC5Db2x1bW4oMikgKyBSMjMgKiBQLkNvbHVtbigzKTtcclxuICAgICAgICAgICAgICAgICAgICBkb3VibGUgcTIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlN1YnByb2JsZW0xKHAyLCB2MiwgSC5Db2x1bW4oMSkpO1xyXG4gICAgICAgICAgICAgICAgICAgIGRvdWJsZVtdIHEyX2FycmF5ID0gbm9ybWFsaXplLkZpbmROb3JtYWxpemVkSm9pbnRzKDEsIG5ld1tdIHsgcTIgfSk7XHJcbiAgICAgICAgICAgICAgICAgICAgaWYgKHEyX2FycmF5Lkxlbmd0aCA9PSAwKSBjb250aW51ZTtcclxuICAgICAgICAgICAgICAgICAgICBxMiA9IHEyX2FycmF5WzBdO1xyXG4gICAgICAgICAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIxMiA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KEguQ29sdW1uKDEpLCBxMik7XHJcbiAgICAgICAgICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUjAzID0gUjAxICogUjEyICogUjIzO1xyXG4gICAgICAgICAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIzNiA9IFIwMy5UcmFuc3Bvc2UoKSAqIGRlc2lyZWRfcG9zZS5SO1xyXG4gICAgICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHY0ID0gUjM2ICogSC5Db2x1bW4oNSk7XHJcbiAgICAgICAgICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcDQgPSBILkNvbHVtbig1KTtcclxuICAgICAgICAgICAgICAgICAgICBkb3VibGVbXSBRNF9RNSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTIocDQsIHY0LCBILkNvbHVtbigzKSwgSC5Db2x1bW4oNCkpO1xyXG4gICAgICAgICAgICAgICAgICAgIGRvdWJsZVtdW10gdGhpcmRfbm9ybWFsaXplID0gbm9ybWFsaXplLkZpbmROb3JtYWxpemVkSm9pbnRzKG5ld1tdIHsgMyw0fSwgUTRfUTUpO1xyXG4gICAgICAgICAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwic2l6ZSBvZiBxcyB7MH0sIHsxfVwiLCB0aGlyZF9ub3JtYWxpemVbMF0uTGVuZ3RoLCB0aGlyZF9ub3JtYWxpemVbMV0uTGVuZ3RoKTtcclxuICAgICAgICAgICAgICAgICAgICBpbnQgbWlub2Z0d28gPSBNYXRoLk1pbih0aGlyZF9ub3JtYWxpemVbMF0uTGVuZ3RoLCB0aGlyZF9ub3JtYWxpemVbMV0uTGVuZ3RoKTtcclxuICAgICAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShtaW5vZnR3byk7XHJcbiAgICAgICAgICAgICAgICAgICAgZm9yIChpbnQgcT0wOyBxIDwgbWlub2Z0d287IHErKylcclxuICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIzNSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KEguQ29sdW1uKDMpLCB0aGlyZF9ub3JtYWxpemVbMF1bcV0pKkdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KEguQ29sdW1uKDQpLCB0aGlyZF9ub3JtYWxpemVbMV1bcV0pO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBSMDUgPSBSMDMgKiBSMzU7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFI1NiA9IFIwNS5UcmFuc3Bvc2UoKSAqIGRlc2lyZWRfcG9zZS5SO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB2NiA9IFI1NiAqIEguQ29sdW1uKDQpO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBwNiA9IEguQ29sdW1uKDQpO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBkb3VibGUgcTYgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlN1YnByb2JsZW0xKHA2LCB2NiwgSC5Db2x1bW4oNSkpO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBkb3VibGVbXSBxNl9hcnJheSA9IG5vcm1hbGl6ZS5GaW5kTm9ybWFsaXplZEpvaW50cyg1LCBuZXdbXSB7IHE2IH0pO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAocTZfYXJyYXkuTGVuZ3RoID09IDApIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBxNiA9IHE2X2FycmF5WzBdO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICBcclxuICAgICAgICAgICAgICAgICAgICAgICAgZG91YmxlW10gdGhldGFfdl9lbnRyeSA9IG5ldyBkb3VibGVbXSB7IHExLCBxMiwgcTMsIHRoaXJkX25vcm1hbGl6ZVswXVtxXSwgdGhpcmRfbm9ybWFsaXplWzFdW3FdLCBxNiB9O1xyXG4gICAgICAgICAgICAgICAgICAgICAgICB0aGV0YV92LkFkZCh0aGV0YV92X2VudHJ5KTtcclxuXHJcblxyXG4gICAgICAgICAgICAgICAgICAgIH1cclxuXHJcblxyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICBpZiAobGFzdF9qb2ludHMgIT0gZGVmYXVsdChkb3VibGVbXSkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGRvdWJsZVtdIHRoZXRhX2Rpc3RfYXJyID0gbmV3IGRvdWJsZVt0aGV0YV92LkNvdW50XTtcclxuXHJcblxyXG4gICAgICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCB0aGV0YV92LkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgICAgICB7XHJcblxyXG4gICAgICAgICAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHRoZXRhX3ZfdmVjID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheSh0aGV0YV92W2ldKTtcclxuICAgICAgICAgICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB0aGV0YV9kaXN0X3ZlYyA9IHRoZXRhX3ZfdmVjLlN1YnRyYWN0KGxhc3Rfam9pbnRzW2ldKTtcclxuICAgICAgICAgICAgICAgICAgICB0aGV0YV9kaXN0X2FycltpXSA9IHRoZXRhX2Rpc3RfdmVjLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgaW50IGluZGV4ID0gMDtcclxuICAgICAgICAgICAgICAgIExpc3Q8ZG91YmxlPiB0aGV0YV9kaXN0X2xpc3QgPSBTeXN0ZW0uTGlucS5FbnVtZXJhYmxlLlRvTGlzdDxkb3VibGU+KHRoZXRhX2Rpc3RfYXJyKTtcclxuICAgICAgICAgICAgICAgIExpc3Q8ZG91YmxlW10+IHJldHVybmVkID0gbmV3IExpc3Q8ZG91YmxlW10+KCk7XHJcbiAgICAgICAgICAgICAgICBmb3IgKGludCB6ID0gMDsgeiA8IHRoZXRhX2Rpc3RfbGlzdC5Db3VudDsgeisrKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGluZGV4ID0gdGhldGFfZGlzdF9saXN0LkluZGV4T2YodGhldGFfZGlzdF9saXN0Lk1pbigpKTtcclxuICAgICAgICAgICAgICAgICAgICByZXR1cm5lZFt6XSA9IHRoZXRhX3ZbaW5kZXhdO1xyXG4gICAgICAgICAgICAgICAgICAgIHRoZXRhX2Rpc3RfbGlzdC5SZW1vdmVBdChpbmRleCk7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICByZXR1cm4gcmV0dXJuZWQuVG9BcnJheSgpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRoZXRhX3YuVG9BcnJheSgpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgIH1cclxuICAgIFxyXG59XHJcbiJdCn0K
