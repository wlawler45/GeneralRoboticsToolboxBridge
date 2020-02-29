Bridge.assembly("BridgeGeneralRoboticsToolbox", function ($asm, globals) {
    "use strict";

    Bridge.define("GeneralRoboticsToolboxTests.UnitTest1", {
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
                AlmostEquals: function (val1, val2, accuracy) {
                    if (accuracy === void 0) { accuracy = 1E-08; }
                    return Math.abs(val1 - val2) < accuracy;
                },
                AlmostEqualsMatrix: function (val1, val2, accuracy) {
                    if (accuracy === void 0) { accuracy = 1E-08; }
                    if (val1.ColumnCount !== val2.ColumnCount || val1.RowCount !== val2.RowCount) {
                        return false;
                    }
                    for (var i = 0; i < val1.RowCount; i = (i + 1) | 0) {
                        for (var j = 0; j < val1.ColumnCount; j = (j + 1) | 0) {
                            if (Math.abs(val1.getItem(i, j) - val2.getItem(i, j)) > accuracy) {

                                return false;
                            }
                        }
                    }

                    return true;
                },
                AlmostEqualsVector: function (val1, val2, accuracy) {
                    if (accuracy === void 0) { accuracy = 1E-08; }
                    if (val1.Count !== val2.Count) {
                        return false;
                    }
                    for (var i = 0; i < val1.Count; i = (i + 1) | 0) {
                        if (Math.abs(val1.getItem(i) - val2.getItem(i)) > accuracy) {
                            return false;
                        }
                    }
                    return true;
                },
                TestHat: function () {
                    System.Console.WriteLine("Testing Hat...");
                    var k = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 2.0, 3.0], System.Double));
                    var khat = GeneralRoboticsToolboxTests.UnitTest1.m_builder.Dense$1(3, 3);
                    khat.setItem(0, 1, -3);
                    khat.setItem(0, 2, 2);
                    khat.setItem(1, 0, 3);
                    khat.setItem(1, 2, -1);
                    khat.setItem(2, 0, -2);
                    khat.setItem(2, 1, 1);
                    var k_hat = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Hat(k);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(k_hat, khat)) {
                        System.Console.WriteLine("hat failed");
                    } else {
                        System.Console.WriteLine("hat succeeded");
                    }
                },
                TestRot: function () {
                    var k = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 0.0, 0.0], System.Double));
                    var rot1 = GeneralRoboticsToolboxTests.UnitTest1.m_builder.Dense$1(3, 3);
                    rot1.setItem(0, 0, 1);
                    rot1.setItem(0, 1, 0);
                    rot1.setItem(0, 2, 0);
                    rot1.setItem(1, 0, 0);
                    rot1.setItem(1, 1, 0);
                    rot1.setItem(1, 2, 1);
                    rot1.setItem(2, 0, 0);
                    rot1.setItem(2, 1, -1);
                    rot1.setItem(2, 2, 0);
                    rot1 = rot1.Transpose();
                    System.Console.WriteLine("Testing Rot...");
                    var rot = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(k, 1.5707963267948966);

                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(rot1, rot, 1E-06)) {
                        System.Console.WriteLine("rot1 failed");
                    } else {
                        System.Console.WriteLine("rot1 succeeded");
                    }

                    var rot2 = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([0, 0, -1.0], System.Double), System.Array.init([0, 1.0, 0], System.Double), System.Array.init([1.0, 0, 0], System.Double)]).Transpose();
                    var k2 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double));
                    var rot_2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(k2, 1.5707963267948966);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(rot2, rot_2, 1E-06)) {
                        System.Console.WriteLine("rot2 failed");
                    } else {
                        System.Console.WriteLine("rot2 succeeded");
                    }


                    var rot3 = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([0, 1.0, 0], System.Double), System.Array.init([-1.0, 0, 0], System.Double), System.Array.init([0, 0, 1.0], System.Double)]).Transpose();
                    var k3 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double));
                    var rot_3 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(k3, 1.5707963267948966);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(rot3, rot_3, 1E-06)) {
                        System.Console.WriteLine("rot3 failed");
                    } else {
                        System.Console.WriteLine("rot3 succeeded");
                    }

                    var rot4 = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.5057639, -0.1340537, 0.8521928], System.Double), System.Array.init([0.6456962, -0.7139224, 0.2709081], System.Double), System.Array.init([0.5720833, 0.6872731, 0.4476342], System.Double)]);
                    var k4 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.4490221, 0.30207945, 0.84090853], System.Double));
                    var rot_4 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(k4, 2.65949884);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(rot4, rot_4, 1E-05)) {
                        System.Console.WriteLine("rot4 failed");
                    } else {
                        System.Console.WriteLine("rot4 succeeded");
                    }
                },
                TestR2Rot: function () {
                    var _R2rot_test = null;

                    _R2rot_test = function (k, theta1) {
                        var R = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(k, theta1);
                        var r2_vals = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.R2rot(R);
                        var _k2 = r2_vals.Item1;
                        var theta2 = r2_vals.Item2;
                        if (Math.abs(theta1 - theta2) > (theta1 + theta2)) {
                            _k2 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_UnaryNegation(_k2);
                            theta2 = -theta2;
                        }

                        if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals(theta1, theta2, 1E-06)) {
                            System.Console.WriteLine("R2Rot failed with theta = " + System.Double.format(theta1));
                        }

                        if (Math.abs(theta1) < 1E-09) {
                            System.Console.WriteLine("R2Rot succeeded");
                            return;
                        }

                        if ((Math.abs(theta1) - Math.PI) < 1E-09) {
                            if ((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(k, _k2)).L2Norm() < 1E-06) {
                                if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(k, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_UnaryNegation(_k2), 1E-06)) {
                                    System.Console.WriteLine("test1 failed");
                                    System.Console.WriteLine("R2Rot failed with -_k2 = " + ((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_UnaryNegation(_k2)).format() || ""));
                                } else {
                                    System.Console.WriteLine("R2Rot succeeded");
                                }

                                return;
                            }

                            if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(k, _k2, 1E-06)) {
                                System.Console.WriteLine("test2 failed");
                                System.Console.WriteLine("R2Rot failed with _k2 = " + (_k2.format() || ""));
                            } else {
                                System.Console.WriteLine("R2Rot succeeded");
                            }

                            return;
                        }

                        if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(k, _k2, 1E-06)) {
                            System.Console.WriteLine("test3 failed");
                            System.Console.WriteLine("R2Rot failed with _k2 = " + (_k2.format() || ""));
                            return;
                        }

                        System.Console.WriteLine("R2Rot succeeded");
                    };

                    _R2rot_test(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 0, 0], System.Double)), 1.5707963267948966);
                    _R2rot_test(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double)), 1.5707963267948966);
                    _R2rot_test(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double)), 1.5707963267948966);
                    _R2rot_test(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.4490221, 0.30207945, 0.84090853], System.Double)), 2.65949884);

                    var k1 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 2.0, 3.0], System.Double)), GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 2.0, 3.0], System.Double)).L2Norm());
                    _R2rot_test(k1, 1E-10);

                    var k2 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([2.0, -1.0, 3.0], System.Double)), GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([2.0, -1.0, 3.0], System.Double)).L2Norm());
                    _R2rot_test(k2, 3.1415926536897931);

                    var k3 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([-2.0, -1.0, 3.0], System.Double)), GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([-2.0, -1.0, 3.0], System.Double)).L2Norm());
                    _R2rot_test(k3, 3.1415926536897931);

                    var k4 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([-2.0, -1.0, 3.0], System.Double)), GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([-2.0, -1.0, 3.0], System.Double)).L2Norm());
                    _R2rot_test(k4, 3.1415926536897931);

                    var k5 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, -1.0, -3.0], System.Double)), GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, -1.0, -3.0], System.Double)).L2Norm());
                    _R2rot_test(k5, 3.1415926536897931);

                    var k6 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double));
                    _R2rot_test(k6, 3.1415926536897931);
                },
                TestScrewMatrix: function () {
                    System.Console.WriteLine("Testing ScrewMatrix");
                    var k = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 2.0, 3.0], System.Double));
                    var G = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Screw_matrix(k);
                    var G_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([1.0, 0, 0, 0, -3, 2], System.Double), System.Array.init([0, 1.0, 0, 3, 0, -1], System.Double), System.Array.init([0, 0, 1.0, -2, 1, 0], System.Double), System.Array.init([0, 0, 0, 1.0, 0, 0], System.Double), System.Array.init([0, 0, 0, 0, 1.0, 0], System.Double), System.Array.init([0, 0, 0, 0, 0, 1.0], System.Double)]);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(G, G_t, 1E-08)) {
                        System.Console.WriteLine("ScrewMarix failed");
                    } else {
                        System.Console.WriteLine("ScrewMatrix succeeded");
                    }
                },
                TestR2Q: function () {
                    var rot = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.5057639, -0.1340537, 0.8521928], System.Double), System.Array.init([0.6456962, -0.7139224, 0.2709081], System.Double), System.Array.init([0.5720833, 0.6872731, 0.4476342], System.Double)]);
                    var q_t = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.2387194, 0.4360402, 0.2933459, 0.8165967], System.Double));
                    var q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.R2Q(rot);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(q, q_t, 1E-06)) {
                        System.Console.WriteLine("R2Q failed");
                    } else {
                        System.Console.WriteLine("R2Q succeeded");
                    }
                },
                TestQ2R: function () {
                    var rot_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.5057639, -0.1340537, 0.8521928], System.Double), System.Array.init([0.6456962, -0.7139224, 0.2709081], System.Double), System.Array.init([0.5720833, 0.6872731, 0.4476342], System.Double)]);
                    var q = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.2387194, 0.4360402, 0.2933459, 0.8165967], System.Double));
                    var rot = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Q2R(q);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(rot, rot_t, 1E-06)) {
                        System.Console.WriteLine("Q2R failed");
                    } else {
                        System.Console.WriteLine("Q2R succeeded");
                    }
                },
                TestRot2Q: function () {
                    var rot = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.R2rot(GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.5057639, -0.1340537, 0.8521928], System.Double), System.Array.init([0.6456962, -0.7139224, 0.2709081], System.Double), System.Array.init([0.5720833, 0.6872731, 0.4476342], System.Double)]));
                    var k = System.Array.init(3, 0, System.Double);
                    for (var i = 0; i < rot.Item1.Count; i = (i + 1) | 0) {
                        k[System.Array.index(i, k)] = rot.Item1.getItem(i);
                    }
                    var theta = rot.Item2;
                    var q_t = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.2387194, 0.4360402, 0.2933459, 0.8165967], System.Double));
                    var q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot2Q(k, theta);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(q, q_t, 1E-06)) {
                        System.Console.WriteLine("Rot2Q failed");
                    } else {
                        System.Console.WriteLine("Rot2Q succeeded");
                    }
                },
                TestQ2Rot: function () {
                    var rot_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.5057639, -0.1340537, 0.8521928], System.Double), System.Array.init([0.6456962, -0.7139224, 0.2709081], System.Double), System.Array.init([0.5720833, 0.6872731, 0.4476342], System.Double)]);
                    var q = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.2387194, 0.4360402, 0.2933459, 0.8165967], System.Double));
                    var rot = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Q2Rot(q);
                    var k = rot.Item1;
                    var theta = rot.Item2;
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(k, theta), rot_t, 1E-06)) {
                        System.Console.WriteLine("Q2Rot failed");
                    } else {
                        System.Console.WriteLine("Q2Rot succeeded");
                    }
                },
                TestQuatcomplement: function () {
                    var q = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.2387194, 0.4360402, 0.2933459, 0.8165967], System.Double));
                    var q_c = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Quatcomplement(q);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals(q.getItem(0), q_c.getItem(0), 1E-08) || !GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(q.SubVector(1, 3), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_UnaryNegation(q_c.SubVector(1, 3)), 1E-08)) {
                        System.Console.WriteLine("Quatcomplement failed");
                    } else {
                        System.Console.WriteLine("Quatcomplement succeeded");
                    }
                },
                TestQuatproduct: function () {
                    var q_1 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.63867877, 0.52251797, 0.56156573, 0.06089615], System.Double));
                    var q_2 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.35764716, 0.61051424, 0.11540801, 0.69716703], System.Double));
                    var R_t = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Q2R(q_1).Multiply$1(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Q2R(q_2));
                    var q_t = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.R2Q(R_t);
                    var q = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Quatproduct(q_1).Multiply$2(q_2);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(q, q_t, 1E-06)) {
                        System.Console.WriteLine("Quatproduct failed");
                    } else {
                        System.Console.WriteLine("Quatproduct succeeded");
                    }
                },
                TestQuatjacobian: function () {
                    var q = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.63867877, 0.52251797, 0.56156573, 0.06089615], System.Double));
                    var J = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Quatjacobian(q);
                    var J_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.26125898, -0.28078286, -0.03044808], System.Double), System.Array.init([0.31933938, 0.03044808, -0.28078286], System.Double), System.Array.init([-0.03044808, 0.31933938, 0.26125898], System.Double), System.Array.init([0.28078286, -0.26125898, 0.31933938], System.Double)]);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(J, J_t, 1E-06)) {
                        System.Console.WriteLine("Quatjacobian failed");
                    } else {
                        System.Console.WriteLine("Quatjacobian succeeded");
                    }
                },
                TestRpy2R: function () {
                    var rpy1 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.17453292519943295, -0.52359877559829882, 1.5707963267948966], System.Double));

                    var R1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rpy2R(rpy1);
                    var R1_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([-0.0, -0.9848077, 0.1736482], System.Double), System.Array.init([0.8660254, -0.0868241, -0.4924039], System.Double), System.Array.init([0.5, 0.1503837, 0.8528686], System.Double)]);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(R1, R1_t, 1E-06)) {
                        System.Console.WriteLine("Rpy2R failed");
                    }
                    var rpy2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.R2Rpy(R1);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(rpy1, rpy2, 1E-06)) {
                        System.Console.WriteLine("Rpy2R failed");
                    }

                    var rpy3 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.17453292519943295, 1.5707963267948966, -0.52359877559829882], System.Double));
                    var R3 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rpy2R(rpy3);

                    System.Console.WriteLine("Rpy2R succeeded");
                },
                Test_Subproblems: function () {
                    var x = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 0, 0], System.Double));
                    var y = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double));
                    var z = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double));

                    if (TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem0(x, y, z) === 1.5707963267948966) {
                        System.Console.WriteLine("Subproblem0 succeeded");
                    } else {
                        System.Console.WriteLine("Subproblem0 failed");
                    }

                    var k1 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(x, z)), (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(x, z)).L2Norm());
                    var k2 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(y, z)), (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(y, z)).L2Norm());
                    if (TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem1(k1, k2, z) === 1.5707963267948966) {
                        System.Console.WriteLine("Subproblem1 succeeded");
                    } else {
                        System.Console.WriteLine("Subproblem1 failed");
                    }

                    var p2 = x;
                    var q2 = x.Add$1(y).Add$1(z);
                    q2 = MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Division$1(q2, q2.L2Norm());
                    var a2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem2(p2, q2, z, y);
                    if (a2.length !== 4) {
                        System.Console.WriteLine("Subproblem2 failed");
                        return;
                    }

                    var r1_0 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a2[System.Array.index(0, a2)]);
                    var r1_1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(y, a2[System.Array.index(1, a2)]);
                    var r1 = (MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(r1_0, r1_1)).Column(0);

                    var r2_0 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a2[System.Array.index(2, a2)]);
                    var r2_1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(y, a2[System.Array.index(3, a2)]);
                    var r2 = (MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(r2_0, r2_1)).Column(0);

                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(r1, q2, 0.0001)) {
                        System.Console.WriteLine("Subproblem2 failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(r2, q2, 0.0001)) {
                        System.Console.WriteLine("Subproblem2 failed");
                        return;
                    }

                    var a3 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem2(x, z, z, y);
                    if (a3.length !== 2) {
                        System.Console.WriteLine("Subproblem2 failed");
                        return;
                    }

                    var r3_0 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a3[System.Array.index(0, a3)]);
                    var r3_1 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(y, a3[System.Array.index(1, a3)]);
                    var r3 = (MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$2(r3_0, r3_1)).Column(0);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(r3, z, 0.0001)) {
                        System.Console.WriteLine("Subproblem2 failed");
                        return;
                    }
                    System.Console.WriteLine("Subproblem2 succeeded");

                    var p4 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.5, 0, 0], System.Double));
                    var q4 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0.75, 0], System.Double));

                    var a4 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem3(p4, q4, z, 0.5);
                    var a5 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem3(p4, q4, z, 1.25);
                    if (a4.length !== 2) {
                        System.Console.WriteLine("Subproblem3 failed");
                        return;
                    }

                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(q4, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a4[System.Array.index(0, a4)]), p4))).L2Norm(), 0.5, 1E-08)) {
                        System.Console.WriteLine("Subproblem3 failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(q4, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a4[System.Array.index(1, a4)]), p4))).L2Norm(), 0.5, 1E-08)) {
                        System.Console.WriteLine("Subproblem3 failed");
                        return;
                    }

                    if (a5.length !== 1) {
                        System.Console.WriteLine("Subproblem3 failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(q4, MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$3(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a5[System.Array.index(0, a5)]), p4))).L2Norm(), 1.25, 1E-08)) {
                        System.Console.WriteLine("Subproblem3 failed");
                        return;
                    }
                    System.Console.WriteLine("Subproblem3 succeeded");
                    var p6 = y;
                    var q6 = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.8, 0.2, 0.5], System.Double));
                    var d6 = 0.3;

                    var a6 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Subproblem4(p6, q6, z, d6);
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$4(p6, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a6[System.Array.index(0, a6)])), q6)), d6, 0.0001)) {
                        System.Console.WriteLine("Subproblem4 failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEquals((MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply(MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply$4(p6, TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(z, a6[System.Array.index(1, a6)])), q6)), d6, 0.0001)) {
                        System.Console.WriteLine("Subproblem4 failed");
                        return;
                    }
                    System.Console.WriteLine("Subproblem4 succeeded");
                },
                RunTests: function () {
                    GeneralRoboticsToolboxTests.UnitTest1.TestRpy2R();
                    GeneralRoboticsToolboxTests.UnitTest1.Test_Subproblems();
                }
            }
        }
    });
});

//# sourceMappingURL=data:application/json;base64,ewogICJ2ZXJzaW9uIjogMywKICAiZmlsZSI6ICJ0ZXN0Lmh0bWwuanMiLAogICJzb3VyY2VSb290IjogIiIsCiAgInNvdXJjZXMiOiBbIlVuaXRUZXN0MS5jcyJdLAogICJuYW1lcyI6IFsiIl0sCiAgIm1hcHBpbmdzIjogIjs7Ozs7Ozs7Ozs7cUNBaUJpREE7cUNBQ0FBOzs7O3dDQUNUQSxNQUFhQSxNQUFhQTs7b0JBRXREQSxPQUFPQSxTQUFTQSxPQUFPQSxRQUFRQTs7OENBRUdBLE1BQXFCQSxNQUFxQkE7O29CQUU1RUEsSUFBSUEscUJBQW9CQSxvQkFBb0JBLGtCQUFpQkE7d0JBQWVBOztvQkFDNUVBLEtBQUtBLFdBQVdBLElBQUlBLGVBQWVBO3dCQUUvQkEsS0FBS0EsV0FBV0EsSUFBSUEsa0JBQWtCQTs0QkFFbENBLElBQUlBLFNBQVNBLGFBQUtBLEdBQUdBLEtBQUtBLGFBQUtBLEdBQUdBLE1BQU1BOztnQ0FFcENBOzs7OztvQkFLWkE7OzhDQUdrQ0EsTUFBcUJBLE1BQXFCQTs7b0JBRTVFQSxJQUFJQSxlQUFjQTt3QkFBWUE7O29CQUM5QkEsS0FBS0EsV0FBV0EsSUFBSUEsWUFBWUE7d0JBRTVCQSxJQUFJQSxTQUFTQSxhQUFLQSxLQUFLQSxhQUFLQSxNQUFNQTs0QkFBVUE7OztvQkFFaERBOzs7b0JBTUFBO29CQUNBQSxRQUFtQkEsNkRBQXVCQTtvQkFDMUNBLFdBQXNCQTtvQkFDdEJBLG1CQUFhQTtvQkFDYkE7b0JBQ0FBO29CQUNBQSxtQkFBYUE7b0JBQ2JBLG1CQUFhQTtvQkFDYkE7b0JBQ0FBLFlBQXVCQSx5REFBMkJBO29CQUNsREEsSUFBSUEsQ0FBQ0EseURBQW1CQSxPQUFPQTt3QkFBU0E7O3dCQUNqQ0E7Ozs7b0JBS1BBLFFBQW1CQSw2REFBdUJBO29CQUMxQ0EsV0FBc0JBO29CQUN0QkE7b0JBQ0FBO29CQUNBQTtvQkFDQUE7b0JBQ0FBO29CQUNBQTtvQkFDQUE7b0JBQ0FBLG1CQUFhQTtvQkFDYkE7b0JBQ0FBLE9BQU9BO29CQUNQQTtvQkFDQUEsVUFBcUJBLHlEQUEyQkEs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LGlFQUFtQ0EsSUFBSUEsSUFBSUE7b0JBQ3pEQSxJQUFJQTt3QkFBa0JBO3dCQUF5Q0E7OztvQkFFL0RBLElBQUlBLENBQUNBLG1EQUFhQSxDQUFDQSx5RUFBS0EsOEhBQTJCQSxHQUFHQSxnQ0FBU0E7d0JBQzdEQTt3QkFBeUNBOztvQkFDM0NBLElBQUlBLENBQUNBLG1EQUFhQSxDQUFDQSx5RUFBS0EsOEhBQTJCQSxHQUFHQSxnQ0FBU0E7d0JBQzdEQTt3QkFBeUNBOzs7b0JBRTNDQSxJQUFJQTt3QkFBa0JBO3dCQUF5Q0E7O29CQUMvREEsSUFBSUEsQ0FBQ0EsbURBQWFBLENBQUNBLHlFQUFLQSw4SEFBMkJBLEdBQUdBLGdDQUFTQTt3QkFDN0RBO3dCQUF5Q0E7O29CQUMzQ0E7b0JBRUFBLFNBQW9CQTtvQkFDcEJBLFNBQW9CQSw2REFBdUJBO29CQUMzQ0E7O29CQUVBQSxTQUFjQSxpRUFBbUNBLElBQUlBLElBQUlBLEdBQUdBO29CQUM1REEsSUFBSUEsQ0FBQ0EsbURBQWFBLENBQUNBLDRJQUFLQSx5REFBMkJBLEdBQUdBLGlDQUFTQSxNQUFLQTt3QkFDbEVBO3dCQUF5Q0E7O29CQUMzQ0EsSUFBSUEsQ0FBQ0EsbURBQWFBLENBQUNBLDRJQUFLQSx5REFBMkJBLEdBQUdBLGlDQUFTQSxNQUFLQTt3QkFDbEVBO3dCQUF5Q0E7O29CQUMzQ0E7OztvQkFnQkFBO29CQUNBQSIsCiAgInNvdXJjZXNDb250ZW50IjogWyJ1c2luZyBTeXN0ZW07XHJcbnVzaW5nIFN5c3RlbS5Db2xsZWN0aW9ucy5HZW5lcmljO1xyXG4vL3VzaW5nIE1pY3Jvc29mdC5WaXN1YWxTdHVkaW8uVGVzdFRvb2xzLlVuaXRUZXN0aW5nO1xyXG51c2luZyBUZXN0R2VuZXJhbFJvYm90aWNzVG9vbGJveE5FVDtcclxudXNpbmcgTWF0aE5ldC5OdW1lcmljcy5MaW5lYXJBbGdlYnJhO1xyXG51c2luZyBNYXRoTmV0Lk51bWVyaWNzLkxpbmVhckFsZ2VicmEuRG91YmxlO1xyXG51c2luZyBNYXRoTmV0Lk51bWVyaWNzLkxpbmVhckFsZ2VicmEuRG91YmxlLk1hdGhOZXQuTnVtZXJpY3MuTGluZWFyQWxnZWJyYTtcclxudXNpbmcgTWF0aE5ldC5OdW1lcmljcztcclxudXNpbmcgQnJpZGdlO1xyXG51c2luZyBCcmlkZ2UuSHRtbDU7XHJcbnVzaW5nIEJyaWRnZS5RVW5pdDtcclxuXHJcbm5hbWVzcGFjZSBHZW5lcmFsUm9ib3RpY3NUb29sYm94VGVzdHNcclxue1xyXG4gICAgW0ZpbGVOYW1lKFwiLi4vc2FtcGxlcy9xdW5pdC9vdXRwdXQvdGVzdC5odG1sXCIpXVxyXG4gICAgcHVibGljIGNsYXNzIFVuaXRUZXN0MVxyXG4gICAge1xyXG4gICAgICAgIHN0YXRpYyBWZWN0b3JCdWlsZGVyPGRvdWJsZT4gdl9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uVmVjdG9yO1xyXG4gICAgICAgIHN0YXRpYyBNYXRyaXhCdWlsZGVyPGRvdWJsZT4gbV9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uTWF0cml4O1xyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgYm9vbCBBbG1vc3RFcXVhbHMoZG91YmxlIHZhbDEsIGRvdWJsZSB2YWwyLCBkb3VibGUgYWNjdXJhY3kgPSAxZS04KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIE1hdGguQWJzKHZhbDEgLSB2YWwyKSA8IGFjY3VyYWN5O1xyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGJvb2wgQWxtb3N0RXF1YWxzTWF0cml4KE1hdHJpeDxkb3VibGU+IHZhbDEsIE1hdHJpeDxkb3VibGU+IHZhbDIsIGRvdWJsZSBhY2N1cmFjeSA9IDFlLTgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodmFsMS5Db2x1bW5Db3VudCAhPSB2YWwyLkNvbHVtbkNvdW50IHx8IHZhbDEuUm93Q291bnQgIT0gdmFsMi5Sb3dDb3VudCkgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICBmb3IgKGludCBpID0gMDsgaSA8IHZhbDEuUm93Q291bnQ7IGkrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgZm9yIChpbnQgaiA9IDA7IGogPCB2YWwxLkNvbHVtbkNvdW50OyBqKyspXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgaWYgKE1hdGguQWJzKHZhbDFbaSwgal0gLSB2YWwyW2ksIGpdKSA+IGFjY3VyYWN5KSB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgYm9vbCBBbG1vc3RFcXVhbHNWZWN0b3IoVmVjdG9yPGRvdWJsZT4gdmFsMSwgVmVjdG9yPGRvdWJsZT4gdmFsMiwgZG91YmxlIGFjY3VyYWN5ID0gMWUtOClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh2YWwxLkNvdW50ICE9IHZhbDIuQ291bnQpIHJldHVybiBmYWxzZTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCB2YWwxLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChNYXRoLkFicyh2YWwxW2ldIC0gdmFsMltpXSkgPiBhY2N1cmFjeSkgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgc3RhdGljIHZvaWQgVGVzdEhhdCgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICAvL1Rlc3QgSGF0XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiVGVzdGluZyBIYXQuLi5cIik7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAyLjAsIDMuMCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4ga2hhdCA9IG1fYnVpbGRlci5EZW5zZSgzLCAzKTtcclxuICAgICAgICAgICAga2hhdFswLCAxXSA9IC0zO1xyXG4gICAgICAgICAgICBraGF0WzAsIDJdID0gMjtcclxuICAgICAgICAgICAga2hhdFsxLCAwXSA9IDM7XHJcbiAgICAgICAgICAgIGtoYXRbMSwgMl0gPSAtMTtcclxuICAgICAgICAgICAga2hhdFsyLCAwXSA9IC0yO1xyXG4gICAgICAgICAgICBraGF0WzIsIDFdID0gMTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4ga19oYXQgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LkhhdChrKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgoa19oYXQsIGtoYXQpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwiaGF0IGZhaWxlZFwiKTsgfVxyXG4gICAgICAgICAgICBlbHNlIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJoYXQgc3VjY2VlZGVkXCIpOyB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0Um90KClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAwLjAsIDAuMCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90MSA9IG1fYnVpbGRlci5EZW5zZSgzLCAzKTtcclxuICAgICAgICAgICAgcm90MVswLCAwXSA9IDE7XHJcbiAgICAgICAgICAgIHJvdDFbMCwgMV0gPSAwO1xyXG4gICAgICAgICAgICByb3QxWzAsIDJdID0gMDtcclxuICAgICAgICAgICAgcm90MVsxLCAwXSA9IDA7XHJcbiAgICAgICAgICAgIHJvdDFbMSwgMV0gPSAwO1xyXG4gICAgICAgICAgICByb3QxWzEsIDJdID0gMTtcclxuICAgICAgICAgICAgcm90MVsyLCAwXSA9IDA7XHJcbiAgICAgICAgICAgIHJvdDFbMiwgMV0gPSAtMTtcclxuICAgICAgICAgICAgcm90MVsyLCAyXSA9IDA7XHJcbiAgICAgICAgICAgIHJvdDEgPSByb3QxLlRyYW5zcG9zZSgpO1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlRlc3RpbmcgUm90Li4uXCIpO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3QgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChrLCBNYXRoLlBJIC8gMi4wKTtcclxuICAgICAgICAgICAgLy9Db25zb2xlLldyaXRlTGluZShrLlRvU3RyaW5nKCkpO1xyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKHJvdDEuVG9TdHJpbmcoKSk7XHJcbiAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUocm90LlRvU3RyaW5nKCkpO1xyXG5cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90MSwgcm90LCAxZS02KSkgeyBDb25zb2xlLldyaXRlTGluZShcInJvdDEgZmFpbGVkXCIpOyB9XHJcbiAgICAgICAgICAgIGVsc2UgeyBDb25zb2xlLldyaXRlTGluZShcInJvdDEgc3VjY2VlZGVkXCIpOyB9XHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3QyID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMobmV3W10geyAwLCAwLCAtMS4wIH0sIG5ld1tdIHsgMCwgMS4wLCAwIH0sIG5ld1tdIHsgMS4wLCAwLCAwIH0pLlRyYW5zcG9zZSgpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrMiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAxLjAsIDAgfSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdF8yID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoazIsIE1hdGguUEkgLyAyKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90Miwgcm90XzIsIDFlLTYpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MiBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MiBzdWNjZWVkZWRcIik7IH1cclxuXHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3QzID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMobmV3W10geyAwLCAxLjAsIDAgfSwgbmV3W10geyAtMS4wLCAwLCAwIH0sIG5ld1tdIHsgMCwgMCwgMS4wIH0pLlRyYW5zcG9zZSgpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrMyA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAwLCAxLjAgfSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdF8zID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoazMsIE1hdGguUEkgLyAyKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90Mywgcm90XzMsIDFlLTYpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MyBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MyBzdWNjZWVkZWRcIik7IH1cclxuXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdDQgPSBtX2J1aWxkZXIuRGVuc2VPZlJvd0FycmF5cyhuZXdbXSB7IC0wLjUwNTc2MzksIC0wLjEzNDA1MzcsIDAuODUyMTkyOCB9LCBuZXdbXSB7IDAuNjQ1Njk2MiwgLTAuNzEzOTIyNCwgMC4yNzA5MDgxIH0sIG5ld1tdIHsgMC41NzIwODMzLCAwLjY4NzI3MzEsIDAuNDQ3NjM0MiB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazQgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC40NDkwMjIxLCAwLjMwMjA3OTQ1LCAwLjg0MDkwODUzIH0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3RfNCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KGs0LCAyLjY1OTQ5ODg0KTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90NCwgcm90XzQsIDFlLTUpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90NCBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90NCBzdWNjZWVkZWRcIik7IH1cclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHN0YXRpYyB2b2lkIFRlc3RSMlJvdCgpXHJcbiAgICAgICAge1xyXG5TeXN0ZW0uQWN0aW9uPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+IF9SMnJvdF90ZXN0ID0gbnVsbDtcbiAgICAgICAgICAgIFxyXG5fUjJyb3RfdGVzdCA9IChrLCB0aGV0YTEpID0+XHJcbntcclxuICAgIE1hdHJpeDxkb3VibGU+IFIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChrLCB0aGV0YTEpO1xyXG4gICAgVHVwbGU8VmVjdG9yPGRvdWJsZT4sIGRvdWJsZT4gcjJfdmFscyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUjJyb3QoUik7XHJcbiAgICBWZWN0b3I8ZG91YmxlPiBfazIgPSByMl92YWxzLkl0ZW0xO1xyXG4gICAgZG91YmxlIHRoZXRhMiA9IHIyX3ZhbHMuSXRlbTI7XHJcbiAgICBpZiAoTWF0aC5BYnModGhldGExIC0gdGhldGEyKSA+ICh0aGV0YTEgKyB0aGV0YTIpKVxyXG4gICAge1xyXG4gICAgICAgIF9rMiA9IC1fazI7XHJcbiAgICAgICAgdGhldGEyID0gLXRoZXRhMjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoIUFsbW9zdEVxdWFscyh0aGV0YTEsIHRoZXRhMiwgMWUtNikpXHJcbiAgICB7XHJcbiAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBmYWlsZWQgd2l0aCB0aGV0YSA9IFwiICsgdGhldGExKTtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoTWF0aC5BYnModGhldGExKSA8IDFlLTkpXHJcbiAgICB7XHJcbiAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICgoTWF0aC5BYnModGhldGExKSAtIE1hdGguUEkpIDwgMWUtOSlcclxuICAgIHtcclxuICAgICAgICBpZiAoKGsgKyBfazIpLkwyTm9ybSgpIDwgMWUtNilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKGssIC1fazIsIDFlLTYpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcInRlc3QxIGZhaWxlZFwiKTtcclxuICAgICAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUoKC1fazIpLlRvU3RyaW5nKCkpO1xyXG4gICAgICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBmYWlsZWQgd2l0aCAtX2syID0gXCIgKyAoLV9rMikuVG9TdHJpbmcoKSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlIyUm90IHN1Y2NlZWRlZFwiKTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgcmV0dXJuO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IoaywgX2syLCAxZS02KSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwidGVzdDIgZmFpbGVkXCIpO1xyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKF9rMi5Ub1N0cmluZygpKTtcclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBmYWlsZWQgd2l0aCBfazIgPSBcIiArIF9rMi5Ub1N0cmluZygpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IoaywgX2syLCAxZS02KSlcclxuICAgIHtcclxuICAgICAgICBDb25zb2xlLldyaXRlTGluZShcInRlc3QzIGZhaWxlZFwiKTtcclxuICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKF9rMi5Ub1N0cmluZygpKTtcclxuICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlIyUm90IGZhaWxlZCB3aXRoIF9rMiA9IFwiICsgX2syLlRvU3RyaW5nKCkpO1xyXG4gICAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBDb25zb2xlLldyaXRlTGluZShcIlIyUm90IHN1Y2NlZWRlZFwiKTtcclxufVxyXG5cclxuO1xuXHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxLjAsIDAsIDAgfSksIE1hdGguUEkgLyAyLjApO1xyXG4gICAgICAgICAgICBfUjJyb3RfdGVzdCh2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMS4wLCAwIH0pLCBNYXRoLlBJIC8gMi4wKTtcclxuICAgICAgICAgICAgX1Iycm90X3Rlc3Qodl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIDAsIDEuMCB9KSwgTWF0aC5QSSAvIDIuMCk7XHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjQ0OTAyMjEsIDAuMzAyMDc5NDUsIDAuODQwOTA4NTMgfSksIDIuNjU5NDk4ODQpO1xyXG5cclxuICAgICAgICAgICAgLy9TaW5ndWxhcml0aWVzXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDEuMCwgMi4wLCAzLjAgfSkgLyB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAyLjAsIDMuMCB9KS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgX1Iycm90X3Rlc3QoazEsIDFlLTEwKTtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsyID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDIuMCwgLTEuMCwgMy4wIH0pIC8gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDIuMCwgLTEuMCwgMy4wIH0pLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICBfUjJyb3RfdGVzdChrMiwgTWF0aC5QSSArIDFlLTEwKTtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGszID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IC0yLjAsIC0xLjAsIDMuMCB9KSAvIHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAtMi4wLCAtMS4wLCAzLjAgfSkuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KGszLCBNYXRoLlBJICsgMWUtMTApO1xyXG5cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazQgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgLTIuMCwgLTEuMCwgMy4wIH0pIC8gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IC0yLjAsIC0xLjAsIDMuMCB9KS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgX1Iycm90X3Rlc3QoazQsIE1hdGguUEkgKyAxZS0xMCk7XHJcblxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrNSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAtMS4wLCAtMy4wIH0pIC8gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIC0xLjAsIC0zLjAgfSkuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KGs1LCBNYXRoLlBJICsgMWUtMTApO1xyXG5cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazYgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMCwgMS4wIH0pO1xyXG4gICAgICAgICAgICBfUjJyb3RfdGVzdChrNiwgTWF0aC5QSSArIDFlLTEwKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHN0YXRpYyB2b2lkIFRlc3RTY3Jld01hdHJpeCgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlRlc3RpbmcgU2NyZXdNYXRyaXhcIik7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAyLjAsIDMuMCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gRyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU2NyZXdfbWF0cml4KGspO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBHX3QgPSBtX2J1aWxkZXIuRGVuc2VPZlJvd0FycmF5cyhcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMS4wLCAwLCAwLCAwLCAtMywgMiB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLCAxLjAsIDAsIDMsIDAsIC0xIH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAsIDAsIDEuMCwgLTIsIDEsIDAgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMCwgMCwgMCwgMS4wLCAwLCAwIH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAsIDAsIDAsIDAsIDEuMCwgMCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLCAwLCAwLCAwLCAwLCAxLjAgfSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzTWF0cml4KEcsIEdfdCwgMWUtOCkpIENvbnNvbGUuV3JpdGVMaW5lKFwiU2NyZXdNYXJpeCBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJTY3Jld01hdHJpeCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UjJRKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC41MDU3NjM5LCAtMC4xMzQwNTM3LCAwLjg1MjE5MjggfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC42NDU2OTYyLCAtMC43MTM5MjI0LCAwLjI3MDkwODEgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC41NzIwODMzLCAwLjY4NzI3MzEsIDAuNDQ3NjM0MiB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcV90ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuMjM4NzE5NCwgMC40MzYwNDAyLCAwLjI5MzM0NTksIDAuODE2NTk2NyB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUjJRKHJvdCk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHEsIHFfdCwgMWUtNikpIENvbnNvbGUuV3JpdGVMaW5lKFwiUjJRIGZhaWxlZFwiKTtcclxuICAgICAgICAgICAgZWxzZSBDb25zb2xlLldyaXRlTGluZShcIlIyUSBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UTJSKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdF90ID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMoXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IC0wLjUwNTc2MzksIC0wLjEzNDA1MzcsIDAuODUyMTkyOCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjY0NTY5NjIsIC0wLjcxMzkyMjQsIDAuMjcwOTA4MSB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjU3MjA4MzMsIDAuNjg3MjczMSwgMC40NDc2MzQyIH0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuMjM4NzE5NCwgMC40MzYwNDAyLCAwLjI5MzM0NTksIDAuODE2NTk2NyB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90ID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RMlIocSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzTWF0cml4KHJvdCwgcm90X3QsIDFlLTYpKSBDb25zb2xlLldyaXRlTGluZShcIlEyUiBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJRMlIgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgc3RhdGljIHZvaWQgVGVzdFJvdDJRKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIFR1cGxlPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+IHJvdCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUjJyb3QobV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMoXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IC0wLjUwNTc2MzksIC0wLjEzNDA1MzcsIDAuODUyMTkyOCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjY0NTY5NjIsIC0wLjcxMzkyMjQsIDAuMjcwOTA4MSB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjU3MjA4MzMsIDAuNjg3MjczMSwgMC40NDc2MzQyIH0pKTtcclxuICAgICAgICAgICAgZG91YmxlW10gayA9IG5ldyBkb3VibGVbM107XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgcm90Lkl0ZW0xLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGtbaV0gPSAoZG91YmxlKXJvdC5JdGVtMVtpXTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBmbG9hdCB0aGV0YSA9IChmbG9hdClyb3QuSXRlbTI7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFfdCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjIzODcxOTQsIDAuNDM2MDQwMiwgMC4yOTMzNDU5LCAwLjgxNjU5NjcgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHEgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdDJRKGssIHRoZXRhKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IocSwgcV90LCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJSb3QyUSBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJSb3QyUSBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UTJSb3QoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90X3QgPSBtX2J1aWxkZXIuRGVuc2VPZlJvd0FycmF5cyhcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgLTAuNTA1NzYzOSwgLTAuMTM0MDUzNywgMC44NTIxOTI4IH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNjQ1Njk2MiwgLTAuNzEzOTIyNCwgMC4yNzA5MDgxIH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNTcyMDgzMywgMC42ODcyNzMxLCAwLjQ0NzYzNDIgfSk7XHJcbiAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUocm90X3QpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuMjM4NzE5NCwgMC40MzYwNDAyLCAwLjI5MzM0NTksIDAuODE2NTk2NyB9KTtcclxuICAgICAgICAgICAgVHVwbGU8VmVjdG9yPGRvdWJsZT4sIGRvdWJsZT4gcm90ID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RMlJvdChxKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gayA9IHJvdC5JdGVtMTtcclxuICAgICAgICAgICAgZG91YmxlIHRoZXRhID0gcm90Lkl0ZW0yO1xyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KGssIHRoZXRhKSk7XHJcbiAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUocm90X3QpO1xyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFsc01hdHJpeChHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChrLCB0aGV0YSksIHJvdF90LCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJRMlJvdCBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJRMlJvdCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UXVhdGNvbXBsZW1lbnQoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjIzODcxOTQsIDAuNDM2MDQwMiwgMC4yOTMzNDU5LCAwLjgxNjU5NjcgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFfYyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUXVhdGNvbXBsZW1lbnQocSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzKHFbMF0sIHFfY1swXSwgMWUtOCkgfHxcclxuICAgICAgICAgICAgICAgICFBbG1vc3RFcXVhbHNWZWN0b3IocS5TdWJWZWN0b3IoMSwgMyksIC1xX2MuU3ViVmVjdG9yKDEsIDMpLCAxZS04KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJRdWF0Y29tcGxlbWVudCBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJRdWF0Y29tcGxlbWVudCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UXVhdHByb2R1Y3QoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcV8xID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuNjM4Njc4NzcsIDAuNTIyNTE3OTcsIDAuNTYxNTY1NzMsIDAuMDYwODk2MTUgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFfMiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjM1NzY0NzE2LCAwLjYxMDUxNDI0LCAwLjExNTQwODAxLCAwLjY5NzE2NzAzIH0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBSX3QgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlEyUihxXzEpLk11bHRpcGx5KEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUTJSKHFfMikpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxX3QgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlIyUShSX3QpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RdWF0cHJvZHVjdChxXzEpLk11bHRpcGx5KHFfMik7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHEsIHFfdCwgMWUtNikpIENvbnNvbGUuV3JpdGVMaW5lKFwiUXVhdHByb2R1Y3QgZmFpbGVkXCIpO1xyXG4gICAgICAgICAgICBlbHNlIENvbnNvbGUuV3JpdGVMaW5lKFwiUXVhdHByb2R1Y3Qgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgIH1cclxuXHJcblxyXG4gICAgICAgIHN0YXRpYyB2b2lkIFRlc3RRdWF0amFjb2JpYW4oKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjYzODY3ODc3LCAwLjUyMjUxNzk3LCAwLjU2MTU2NTczLCAwLjA2MDg5NjE1IH0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBKID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RdWF0amFjb2JpYW4ocSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IEpfdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKG5ld1tdIHsgLTAuMjYxMjU4OTgsIC0wLjI4MDc4Mjg2LCAtMC4wMzA0NDgwOCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjMxOTMzOTM4LCAwLjAzMDQ0ODA4LCAtMC4yODA3ODI4NiB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC4wMzA0NDgwOCwgMC4zMTkzMzkzOCwgMC4yNjEyNTg5OCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjI4MDc4Mjg2LCAtMC4yNjEyNTg5OCwgMC4zMTkzMzkzOCB9KTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgoSiwgSl90LCAxZS02KSkgeyBDb25zb2xlLldyaXRlTGluZShcIlF1YXRqYWNvYmlhbiBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSBDb25zb2xlLldyaXRlTGluZShcIlF1YXRqYWNvYmlhbiBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UnB5MlIoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcnB5MSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxMCAqIE1hdGguUEkgLyAxODAsIC0zMCAqIE1hdGguUEkgLyAxODAsIDkwICogTWF0aC5QSSAvIDE4MCB9KTtcclxuXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIxID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5ScHkyUihycHkxKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUjFfdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC4wMDAwMDAwLCAtMC45ODQ4MDc3LCAwLjE3MzY0ODIgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC44NjYwMjU0LCAtMC4wODY4MjQxLCAtMC40OTI0MDM5IH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNTAwMDAwMCwgMC4xNTAzODM3LCAwLjg1Mjg2ODYgfSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzTWF0cml4KFIxLCBSMV90LCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJScHkyUiBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHJweTIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlIyUnB5KFIxKTtcclxuICAgICAgICAgICAgLy9Db25zb2xlLldyaXRlTGluZShycHkyKTtcclxuICAgICAgICAgICAgaWYoIUFsbW9zdEVxdWFsc1ZlY3RvcihycHkxLCBycHkyLCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJScHkyUiBmYWlsZWRcIik7XHJcblxyXG4gICAgICAgICAgICAvLyBDaGVjayBzaW5ndWxhcml0eVxyXG4gICAgICAgICAgICAvLyBOT1RFOiBEb2VzIG5vdCByYWlzZSBleGNlcHRpb25cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcnB5MyA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxMCAqIE1hdGguUEkgLyAxODAsIDkwICogTWF0aC5QSSAvIDE4MCwgLTMwICogTWF0aC5QSSAvIDE4MCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUjMgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJweTJSKHJweTMpO1xyXG5cclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJScHkyUiBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0X1N1YnByb2JsZW1zKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHggPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAwLCAwIH0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB5ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIDEuMCwgMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4geiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAwLCAxLjAgfSk7XHJcblxyXG4gICAgICAgICAgICAvLyBTdWJwcm9ibGVtMFxyXG4gICAgICAgICAgICBpZiAoR2VuZXJhbFJvYm90aWNzVG9vbGJveC5TdWJwcm9ibGVtMCh4LCB5LCB6KSA9PSBNYXRoLlBJIC8gMikgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMCBmYWlsZWRcIik7XHJcblxyXG4gICAgICAgICAgICAvLyBTdWJwcm9ibGVtMVxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrMSA9ICh4ICsgeikgLyAoeCArIHopLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrMiA9ICh5ICsgeikgLyAoeSArIHopLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICBpZiAoR2VuZXJhbFJvYm90aWNzVG9vbGJveC5TdWJwcm9ibGVtMShrMSwgazIsIHopID09IE1hdGguUEkgLyAyKSBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0xIHN1Y2NlZWRlZFwiKTtcclxuICAgICAgICAgICAgZWxzZSBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0xIGZhaWxlZFwiKTtcclxuXHJcbiAgICAgICAgICAgIC8vIFN1YnByb2JsZW0yXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHAyID0geDtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcTIgPSB4LkFkZCh5KS5BZGQoeik7XHJcbiAgICAgICAgICAgIHEyID0gcTIgLyBxMi5MMk5vcm0oKTtcclxuICAgICAgICAgICAgZG91YmxlW10gYTIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlN1YnByb2JsZW0yKHAyLCBxMiwgeiwgeSk7XHJcbiAgICAgICAgICAgIGlmIChhMi5MZW5ndGggIT0gNCkgeyBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0yIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcbiAgICAgICAgICAgIC8vTk9URTogRElGRkVSRU5UIFRIQU4gUFlUSE9OIFZFUlNJT05cclxuXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHIxXzAgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdCh6LCBhMlswXSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHIxXzEgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdCh5LCBhMlsxXSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHIxID0gKHIxXzAgKiByMV8xKS5Db2x1bW4oMCk7XHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByMl8wID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTJbMl0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByMl8xID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeSwgYTJbM10pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiByMiA9IChyMl8wICogcjJfMSkuQ29sdW1uKDApO1xyXG5cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IocjEsIHEyLCAxZS00KSkgeyBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0yIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHIyLCBxMiwgMWUtNCkpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG5cclxuICAgICAgICAgICAgZG91YmxlW10gYTMgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlN1YnByb2JsZW0yKHgsIHosIHosIHkpO1xyXG4gICAgICAgICAgICBpZiAoYTMuTGVuZ3RoICE9IDIpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICAvL05PVEU6IERJRkZFUkVOVCBUSEFOIFBZVEhPTiBWRVJTSU9OXHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByM18wID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTNbMF0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByM18xID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeSwgYTNbMV0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiByMyA9IChyM18wICogcjNfMSkuQ29sdW1uKDApO1xyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFsc1ZlY3RvcihyMywgeiwgMWUtNCkpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0yIHN1Y2NlZWRlZFwiKTtcclxuXHJcbiAgICAgICAgICAgIC8vIFN1YnByb2JsZW0zXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHA0ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IC41LCAwLCAwIH0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxNCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAuNzUsIDAgfSk7XHJcblxyXG4gICAgICAgICAgICBkb3VibGVbXSBhNCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTMocDQsIHE0LCB6LCAuNSk7XHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGE1ID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5TdWJwcm9ibGVtMyhwNCwgcTQsIHosIDEuMjUpO1xyXG4gICAgICAgICAgICBpZiAoYTQuTGVuZ3RoICE9IDIpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMyBmYWlsZWRcIik7IHJldHVybjsgfVxyXG5cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHMoKHE0ICsgR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTRbMF0pICogcDQpLkwyTm9ybSgpLCAwLjUsIDFlLTgpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTMgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHMoKHE0ICsgR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTRbMV0pICogcDQpLkwyTm9ybSgpLCAwLjUsIDFlLTgpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTMgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuXHJcbiAgICAgICAgICAgIGlmIChhNS5MZW5ndGggIT0gMSkgeyBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0zIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzKChxNCArIEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHosIGE1WzBdKSAqIHA0KS5MMk5vcm0oKSwgMS4yNSwgMWUtOCkpXHJcbiAgICAgICAgICAgIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMyBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0zIHN1Y2NlZWRlZFwiKTtcclxuICAgICAgICAgICAgLy8gU3VicHJvYmxlbTRcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcDYgPSB5O1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxNiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAuOCwgLjIsIC41IH0pO1xyXG4gICAgICAgICAgICBkb3VibGUgZDYgPSAuMztcclxuXHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGE2ID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5TdWJwcm9ibGVtNChwNiwgcTYsIHosIGQ2KTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHMoKHA2ICogR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTZbMF0pICogcTYpLCBkNiwgMWUtNCkpXHJcbiAgICAgICAgICAgIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtNCBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFscygocDYgKiBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdCh6LCBhNlsxXSkgKiBxNiksIGQ2LCAxZS00KSlcclxuICAgICAgICAgICAgeyBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW00IGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTQgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcHVibGljIHN0YXRpYyB2b2lkIFJ1blRlc3RzKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIC8vVGVzdEhhdCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RSb3QoKTtcclxuICAgICAgICAgICAgLy9UZXN0UjJSb3QoKTtcclxuICAgICAgICAgICAgLy9UZXN0U2NyZXdNYXRyaXgoKTtcclxuICAgICAgICAgICAgLy9UZXN0UjJRKCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFEyUigpO1xyXG4gICAgICAgICAgICAvL1Rlc3RSb3QyUSgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RRMlJvdCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RRdWF0Y29tcGxlbWVudCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RRdWF0cHJvZHVjdCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RRdWF0amFjb2JpYW4oKTtcclxuICAgICAgICAgICAgVGVzdFJweTJSKCk7XHJcbiAgICAgICAgICAgIFRlc3RfU3VicHJvYmxlbXMoKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbn1cclxuIl0KfQo=
