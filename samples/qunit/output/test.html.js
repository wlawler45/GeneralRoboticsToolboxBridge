Bridge.assembly("BridgeGeneralRoboticsToolbox", function ($asm, globals) {
    "use strict";

    Bridge.define("GeneralRoboticsToolboxTests.UnitTest1", {
        statics: {
            fields: {
                in_2_m: 0,
                v_builder: null,
                m_builder: null
            },
            ctors: {
                init: function () {
                    this.in_2_m = 0.0254;
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
                    System.Console.WriteLine(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.R2Rpy(R3).format());

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
                puma260b_robot: function () {


                    var x = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 0, 0], System.Double));
                    var y = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double));
                    var z = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double));
                    var a = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.0, 0, 0], System.Double));

                    var H = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfColumnVectors([z, y, y, z, y, x]);
                    var P = MathNet.Numerics.LinearAlgebra.Matrix$1(System.Double).op_Multiply(0.0254, GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfColumnVectors([MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(13, z), a, (MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Subtraction$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Addition$2(MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(-4.9, y), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(7.8, x)), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(0.75, z))), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(-8.0, z), a, a, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(2.2, x)]));
                    var joint_type = System.Array.init([0, 0, 0, 0, 0, 0], System.Int32);
                    var joint_min = System.Array.init([-5.0, -256, -214, -384, -32, -267], System.Double);
                    var joint_max = System.Array.init([313.0, 76, 34, 194, 212, 267], System.Double);
                    for (var i = 0; i < joint_min.length; i = (i + 1) | 0) {
                        joint_min[System.Array.index(i, joint_min)] = joint_min[System.Array.index(i, joint_min)] * Math.PI / 180.0;
                        joint_max[System.Array.index(i, joint_max)] = joint_max[System.Array.index(i, joint_max)] * Math.PI / 180.0;
                    }
                    return new TestGeneralRoboticsToolboxNET.Robot.$ctor1(H, P, joint_type, joint_min, joint_max);
                },
                abb_irb6640_180_255_robot: function () {


                    var x = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([1.0, 0, 0], System.Double));
                    var y = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double));
                    var z = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double));
                    var a = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.0, 0, 0], System.Double));

                    var H = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfColumnVectors([z, y, y, x, y, x]);
                    var P = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfColumnVectors([MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(0.78, z), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(0.32, x), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(1.075, z), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(0.2, z), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(1.142, x), MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$1(0.2, x), a]);
                    var joint_type = System.Array.init([0, 0, 0, 0, 0, 0], System.Int32);
                    var joint_min = System.Array.init([-170.0, -65, -180, -300, -120, -360], System.Double);
                    var joint_max = System.Array.init([170.0, 85, 70, 300, 120, 360], System.Double);
                    for (var i = 0; i < joint_min.length; i = (i + 1) | 0) {
                        joint_min[System.Array.index(i, joint_min)] = joint_min[System.Array.index(i, joint_min)] * Math.PI / 180.0;
                        joint_max[System.Array.index(i, joint_max)] = joint_max[System.Array.index(i, joint_max)] * Math.PI / 180.0;
                    }
                    return new TestGeneralRoboticsToolboxNET.Robot.$ctor1(H, P, joint_type, joint_min, joint_max);
                },
                puma260b_robot_tool: function () {
                    var robot = GeneralRoboticsToolboxTests.UnitTest1.puma260b_robot();
                    robot.R_tool = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double)), 1.5707963267948966);
                    robot.P_tool = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.05, 0, 0], System.Double));
                    return robot;
                },
                TestFwdkin: function () {
                    var puma = GeneralRoboticsToolboxTests.UnitTest1.puma260b_robot();

                    var pose = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Fwdkin(puma, System.Array.init([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], System.Double));
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(pose.R, GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseIdentity(3), 1E-08)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(pose.P, GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([10.0 * GeneralRoboticsToolboxTests.UnitTest1.in_2_m, -4.9 * GeneralRoboticsToolboxTests.UnitTest1.in_2_m, 4.25 * GeneralRoboticsToolboxTests.UnitTest1.in_2_m], System.Double)), 1E-06)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }

                    var joints2 = System.Array.init([180.0, -90, -90, 90, 90, 90], System.Double);
                    for (var i = 0; i < joints2.length; i = (i + 1) | 0) {
                        joints2[System.Array.index(i, joints2)] = joints2[System.Array.index(i, joints2)] * Math.PI / 180.0;
                    }
                    var pose2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Fwdkin(puma, joints2);
                    var rot2 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 0, 1.0], System.Double)), Math.PI).Multiply$1(TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Rot(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0, 1.0, 0], System.Double)), -1.5707963267948966));
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(pose2.R, rot2, 1E-06)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(pose2.P, MathNet.Numerics.LinearAlgebra.Vector$1(System.Double).op_Multiply$2(GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([-0.75, 4.9, 31], System.Double)), 0.0254), 1E-06)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }

                    var joints3 = System.Array.init([50.0, -105, 31, 4, 126, -184], System.Double);
                    for (var i1 = 0; i1 < joints3.length; i1 = (i1 + 1) | 0) {
                        joints3[System.Array.index(i1, joints3)] = joints3[System.Array.index(i1, joints3)] * Math.PI / 180.0;
                    }
                    var pose3 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Fwdkin(puma, joints3);
                    var pose3_R_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([0.4274, 0.8069, -0.4076], System.Double), System.Array.init([0.4455, -0.5804, -0.6817], System.Double), System.Array.init([-0.7866, 0.1097, -0.6076], System.Double)]);

                    var pose3_P_t = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.2236, 0.0693, 0.4265], System.Double));
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(pose3.R, pose3_R_t, 0.0001)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(pose3.P, pose3_P_t, 0.0001)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }

                    var puma_tool = GeneralRoboticsToolboxTests.UnitTest1.puma260b_robot_tool();

                    var pose4 = TestGeneralRoboticsToolboxNET.GeneralRoboticsToolbox.Fwdkin(puma_tool, joints3);
                    var pose4_R_t = GeneralRoboticsToolboxTests.UnitTest1.m_builder.DenseOfRowArrays([System.Array.init([0.4076, 0.8069, 0.4274], System.Double), System.Array.init([0.681654, -0.580357, 0.44557], System.Double), System.Array.init([0.60759, 0.1097, -0.7866], System.Double)]);
                    System.Console.WriteLine(System.String.format("Robot R tool={0}", pose4.R));
                    System.Console.WriteLine(System.String.format("Robot R calculated tool={0}", pose4_R_t));
                    System.Console.WriteLine(System.String.format("Robot p tool={0}", pose4.P));

                    var pose4_P_t = GeneralRoboticsToolboxTests.UnitTest1.v_builder.DenseOfArray(System.Array.init([0.245, 0.0916, 0.3872], System.Double));
                    System.Console.WriteLine(System.String.format("Robot p calculated tool={0}", pose4_P_t));
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsMatrix(pose4.R, pose4_R_t, 14)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }
                    if (!GeneralRoboticsToolboxTests.UnitTest1.AlmostEqualsVector(pose4.P, pose4_P_t, 14)) {
                        System.Console.WriteLine("FwdKin failed");
                        return;
                    }
                    System.Console.WriteLine("FwdKin succeeded");
                },
                RunTests: function () {
                    GeneralRoboticsToolboxTests.UnitTest1.TestRpy2R();
                    GeneralRoboticsToolboxTests.UnitTest1.Test_Subproblems();
                    GeneralRoboticsToolboxTests.UnitTest1.TestFwdkin();
                }
            }
        }
    });
});

//# sourceMappingURL=data:application/json;base64,ewogICJ2ZXJzaW9uIjogMywKICAiZmlsZSI6ICJ0ZXN0Lmh0bWwuanMiLAogICJzb3VyY2VSb290IjogIiIsCiAgInNvdXJjZXMiOiBbIlVuaXRUZXN0MS5jcyJdLAogICJuYW1lcyI6IFsiIl0sCiAgIm1hcHBpbmdzIjogIjs7Ozs7Ozs7Ozs7OztxQ0FtQmlEQTtxQ0FDQUE7Ozs7d0NBQ1RBLE1BQWFBLE1BQWFBOztvQkFFdERBLE9BQU9BLFNBQVNBLE9BQU9BLFFBQVFBOzs4Q0FFR0EsTUFBcUJBLE1BQXFCQTs7b0JBRTVFQSxJQUFJQSxxQkFBb0JBLG9CQUFvQkEsa0JBQWlCQTt3QkFBZUE7O29CQUM1RUEsS0FBS0EsV0FBV0EsSUFBSUEsZUFBZUE7d0JBRS9CQSxLQUFLQSxXQUFXQSxJQUFJQSxrQkFBa0JBOzRCQUVsQ0EsSUFBSUEsU0FBU0EsYUFBS0EsR0FBR0EsS0FBS0EsYUFBS0EsR0FBR0EsTUFBTUE7O2dDQUVwQ0E7Ozs7O29CQUtaQTs7OENBR2tDQSxNQUFxQkEsTUFBcUJBOztvQkFFNUVBLElBQUlBLGVBQWNBO3dCQUFZQTs7b0JBQzlCQSxLQUFLQSxXQUFXQSxJQUFJQSxZQUFZQTt3QkFFNUJBLElBQUlBLFNBQVNBLGFBQUtBLEtBQUtBLGFBQUtBLE1BQU1BOzRCQUFVQTs7O29CQUVoREE7OztvQkFNQUE7b0JBQ0FBLFFBQW1CQSw2REFBdUJBO29CQUMxQ0EsV0FBc0JBO29CQUN0QkEsbUJBQWFBO29CQUNiQTtvQkFDQUE7b0JBQ0FBLG1CQUFhQTtvQkFDYkEsbUJBQWFBO29CQUNiQTtvQkFDQUEsWUFBdUJBLHlEQUEyQkE7b0JBQ2xEQSxJQUFJQSxDQUFDQSx5REFBbUJBLE9BQU9BO3dCQUFTQTs7d0JBQ2pDQTs7OztvQkFLUEEsUUFBbUJBLDZEQUF1QkE7b0JBQzFDQSxXQUFzQkE7b0JBQ3RCQTtvQkFDQUE7b0JBQ0FBO29CQUNBQTtvQkFDQUE7b0JBQ0FBO29CQUNBQTtvQkFDQUEsbUJBQWFBO29CQUNiQTtvQkFDQUEsT0FBT0E7b0JBQ1BBO29CQUNBQSxVQUFxQkEseURBQTJC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QWtCQSxxQ0FDbEJBLG9DQUF5QkE7b0JBQzdCQSxrRUFBc0NBO29CQUN0Q0EsNkVBQWlEQTtvQkFDakRBLGtFQUFzQ0E7O29CQUV0Q0EsZ0JBQTJCQSw2REFBdUJBO29CQUNsREEsNkVBQWlEQTtvQkFDakRBLElBQUdBLENBQUNBLHlEQUFtQkEsU0FBU0EsV0FBV0E7d0JBQ3pDQTt3QkFBb0NBOztvQkFDdENBLElBQUlBLENBQUNBLHlEQUFtQkEsU0FBU0EsV0FBV0E7d0JBQzFDQTt3QkFBb0NBOztvQkFDdENBOzs7b0JBZ0JBQTtvQkFDQUE7b0JBQ0FBIiwKICAic291cmNlc0NvbnRlbnQiOiBbInVzaW5nIFN5c3RlbTtcclxudXNpbmcgU3lzdGVtLkNvbGxlY3Rpb25zLkdlbmVyaWM7XHJcbi8vdXNpbmcgTWljcm9zb2Z0LlZpc3VhbFN0dWRpby5UZXN0VG9vbHMuVW5pdFRlc3Rpbmc7XHJcbnVzaW5nIFRlc3RHZW5lcmFsUm9ib3RpY3NUb29sYm94TkVUO1xyXG51c2luZyBNYXRoTmV0Lk51bWVyaWNzLkxpbmVhckFsZ2VicmE7XHJcbnVzaW5nIE1hdGhOZXQuTnVtZXJpY3MuTGluZWFyQWxnZWJyYS5Eb3VibGU7XHJcbnVzaW5nIE1hdGhOZXQuTnVtZXJpY3MuTGluZWFyQWxnZWJyYS5Eb3VibGUuTWF0aE5ldC5OdW1lcmljcy5MaW5lYXJBbGdlYnJhO1xyXG51c2luZyBNYXRoTmV0Lk51bWVyaWNzO1xyXG51c2luZyBCcmlkZ2U7XHJcbnVzaW5nIEJyaWRnZS5IdG1sNTtcclxudXNpbmcgQnJpZGdlLlFVbml0O1xyXG5cclxubmFtZXNwYWNlIEdlbmVyYWxSb2JvdGljc1Rvb2xib3hUZXN0c1xyXG57XHJcbiAgICBbRmlsZU5hbWUoXCIuLi9zYW1wbGVzL3F1bml0L291dHB1dC90ZXN0Lmh0bWxcIildXHJcbiAgICBwdWJsaWMgY2xhc3MgVW5pdFRlc3QxXHJcbiAgICB7XHJcbiAgICAgICAgc3RhdGljIGRvdWJsZSBpbl8yX20gPSAwLjAyNTQ7XHJcblxyXG4gICAgICAgIHN0YXRpYyBWZWN0b3JCdWlsZGVyPGRvdWJsZT4gdl9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uVmVjdG9yO1xyXG4gICAgICAgIHN0YXRpYyBNYXRyaXhCdWlsZGVyPGRvdWJsZT4gbV9idWlsZGVyID0gQnVpbGRlckluc3RhbmNlPGRvdWJsZT4uTWF0cml4O1xyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgYm9vbCBBbG1vc3RFcXVhbHMoZG91YmxlIHZhbDEsIGRvdWJsZSB2YWwyLCBkb3VibGUgYWNjdXJhY3kgPSAxZS04KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIE1hdGguQWJzKHZhbDEgLSB2YWwyKSA8IGFjY3VyYWN5O1xyXG4gICAgICAgIH1cclxuICAgICAgICBwdWJsaWMgc3RhdGljIGJvb2wgQWxtb3N0RXF1YWxzTWF0cml4KE1hdHJpeDxkb3VibGU+IHZhbDEsIE1hdHJpeDxkb3VibGU+IHZhbDIsIGRvdWJsZSBhY2N1cmFjeSA9IDFlLTgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodmFsMS5Db2x1bW5Db3VudCAhPSB2YWwyLkNvbHVtbkNvdW50IHx8IHZhbDEuUm93Q291bnQgIT0gdmFsMi5Sb3dDb3VudCkgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICBmb3IgKGludCBpID0gMDsgaSA8IHZhbDEuUm93Q291bnQ7IGkrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgZm9yIChpbnQgaiA9IDA7IGogPCB2YWwxLkNvbHVtbkNvdW50OyBqKyspXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgaWYgKE1hdGguQWJzKHZhbDFbaSwgal0gLSB2YWwyW2ksIGpdKSA+IGFjY3VyYWN5KSB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHB1YmxpYyBzdGF0aWMgYm9vbCBBbG1vc3RFcXVhbHNWZWN0b3IoVmVjdG9yPGRvdWJsZT4gdmFsMSwgVmVjdG9yPGRvdWJsZT4gdmFsMiwgZG91YmxlIGFjY3VyYWN5ID0gMWUtOClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh2YWwxLkNvdW50ICE9IHZhbDIuQ291bnQpIHJldHVybiBmYWxzZTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCB2YWwxLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmIChNYXRoLkFicyh2YWwxW2ldIC0gdmFsMltpXSkgPiBhY2N1cmFjeSkgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgc3RhdGljIHZvaWQgVGVzdEhhdCgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICAvL1Rlc3QgSGF0XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiVGVzdGluZyBIYXQuLi5cIik7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAyLjAsIDMuMCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4ga2hhdCA9IG1fYnVpbGRlci5EZW5zZSgzLCAzKTtcclxuICAgICAgICAgICAga2hhdFswLCAxXSA9IC0zO1xyXG4gICAgICAgICAgICBraGF0WzAsIDJdID0gMjtcclxuICAgICAgICAgICAga2hhdFsxLCAwXSA9IDM7XHJcbiAgICAgICAgICAgIGtoYXRbMSwgMl0gPSAtMTtcclxuICAgICAgICAgICAga2hhdFsyLCAwXSA9IC0yO1xyXG4gICAgICAgICAgICBraGF0WzIsIDFdID0gMTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4ga19oYXQgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LkhhdChrKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgoa19oYXQsIGtoYXQpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwiaGF0IGZhaWxlZFwiKTsgfVxyXG4gICAgICAgICAgICBlbHNlIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJoYXQgc3VjY2VlZGVkXCIpOyB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0Um90KClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAwLjAsIDAuMCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90MSA9IG1fYnVpbGRlci5EZW5zZSgzLCAzKTtcclxuICAgICAgICAgICAgcm90MVswLCAwXSA9IDE7XHJcbiAgICAgICAgICAgIHJvdDFbMCwgMV0gPSAwO1xyXG4gICAgICAgICAgICByb3QxWzAsIDJdID0gMDtcclxuICAgICAgICAgICAgcm90MVsxLCAwXSA9IDA7XHJcbiAgICAgICAgICAgIHJvdDFbMSwgMV0gPSAwO1xyXG4gICAgICAgICAgICByb3QxWzEsIDJdID0gMTtcclxuICAgICAgICAgICAgcm90MVsyLCAwXSA9IDA7XHJcbiAgICAgICAgICAgIHJvdDFbMiwgMV0gPSAtMTtcclxuICAgICAgICAgICAgcm90MVsyLCAyXSA9IDA7XHJcbiAgICAgICAgICAgIHJvdDEgPSByb3QxLlRyYW5zcG9zZSgpO1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlRlc3RpbmcgUm90Li4uXCIpO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3QgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChrLCBNYXRoLlBJIC8gMi4wKTtcclxuICAgICAgICAgICAgLy9Db25zb2xlLldyaXRlTGluZShrLlRvU3RyaW5nKCkpO1xyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKHJvdDEuVG9TdHJpbmcoKSk7XHJcbiAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUocm90LlRvU3RyaW5nKCkpO1xyXG5cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90MSwgcm90LCAxZS02KSkgeyBDb25zb2xlLldyaXRlTGluZShcInJvdDEgZmFpbGVkXCIpOyB9XHJcbiAgICAgICAgICAgIGVsc2UgeyBDb25zb2xlLldyaXRlTGluZShcInJvdDEgc3VjY2VlZGVkXCIpOyB9XHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3QyID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMobmV3W10geyAwLCAwLCAtMS4wIH0sIG5ld1tdIHsgMCwgMS4wLCAwIH0sIG5ld1tdIHsgMS4wLCAwLCAwIH0pLlRyYW5zcG9zZSgpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrMiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAxLjAsIDAgfSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdF8yID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoazIsIE1hdGguUEkgLyAyKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90Miwgcm90XzIsIDFlLTYpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MiBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MiBzdWNjZWVkZWRcIik7IH1cclxuXHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3QzID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMobmV3W10geyAwLCAxLjAsIDAgfSwgbmV3W10geyAtMS4wLCAwLCAwIH0sIG5ld1tdIHsgMCwgMCwgMS4wIH0pLlRyYW5zcG9zZSgpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrMyA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAwLCAxLjAgfSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdF8zID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoazMsIE1hdGguUEkgLyAyKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90Mywgcm90XzMsIDFlLTYpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MyBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90MyBzdWNjZWVkZWRcIik7IH1cclxuXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdDQgPSBtX2J1aWxkZXIuRGVuc2VPZlJvd0FycmF5cyhuZXdbXSB7IC0wLjUwNTc2MzksIC0wLjEzNDA1MzcsIDAuODUyMTkyOCB9LCBuZXdbXSB7IDAuNjQ1Njk2MiwgLTAuNzEzOTIyNCwgMC4yNzA5MDgxIH0sIG5ld1tdIHsgMC41NzIwODMzLCAwLjY4NzI3MzEsIDAuNDQ3NjM0MiB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazQgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC40NDkwMjIxLCAwLjMwMjA3OTQ1LCAwLjg0MDkwODUzIH0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByb3RfNCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KGs0LCAyLjY1OTQ5ODg0KTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocm90NCwgcm90XzQsIDFlLTUpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90NCBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSB7IENvbnNvbGUuV3JpdGVMaW5lKFwicm90NCBzdWNjZWVkZWRcIik7IH1cclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHN0YXRpYyB2b2lkIFRlc3RSMlJvdCgpXHJcbiAgICAgICAge1xyXG5TeXN0ZW0uQWN0aW9uPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+IF9SMnJvdF90ZXN0ID0gbnVsbDtcbiAgICAgICAgICAgIFxyXG5fUjJyb3RfdGVzdCA9IChrLCB0aGV0YTEpID0+XHJcbntcclxuICAgIE1hdHJpeDxkb3VibGU+IFIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChrLCB0aGV0YTEpO1xyXG4gICAgVHVwbGU8VmVjdG9yPGRvdWJsZT4sIGRvdWJsZT4gcjJfdmFscyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUjJyb3QoUik7XHJcbiAgICBWZWN0b3I8ZG91YmxlPiBfazIgPSByMl92YWxzLkl0ZW0xO1xyXG4gICAgZG91YmxlIHRoZXRhMiA9IHIyX3ZhbHMuSXRlbTI7XHJcbiAgICBpZiAoTWF0aC5BYnModGhldGExIC0gdGhldGEyKSA+ICh0aGV0YTEgKyB0aGV0YTIpKVxyXG4gICAge1xyXG4gICAgICAgIF9rMiA9IC1fazI7XHJcbiAgICAgICAgdGhldGEyID0gLXRoZXRhMjtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoIUFsbW9zdEVxdWFscyh0aGV0YTEsIHRoZXRhMiwgMWUtNikpXHJcbiAgICB7XHJcbiAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBmYWlsZWQgd2l0aCB0aGV0YSA9IFwiICsgdGhldGExKTtcclxuICAgIH1cclxuXHJcbiAgICBpZiAoTWF0aC5BYnModGhldGExKSA8IDFlLTkpXHJcbiAgICB7XHJcbiAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIGlmICgoTWF0aC5BYnModGhldGExKSAtIE1hdGguUEkpIDwgMWUtOSlcclxuICAgIHtcclxuICAgICAgICBpZiAoKGsgKyBfazIpLkwyTm9ybSgpIDwgMWUtNilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKGssIC1fazIsIDFlLTYpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcInRlc3QxIGZhaWxlZFwiKTtcclxuICAgICAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUoKC1fazIpLlRvU3RyaW5nKCkpO1xyXG4gICAgICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBmYWlsZWQgd2l0aCAtX2syID0gXCIgKyAoLV9rMikuVG9TdHJpbmcoKSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlIyUm90IHN1Y2NlZWRlZFwiKTtcclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgcmV0dXJuO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IoaywgX2syLCAxZS02KSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwidGVzdDIgZmFpbGVkXCIpO1xyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKF9rMi5Ub1N0cmluZygpKTtcclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBmYWlsZWQgd2l0aCBfazIgPSBcIiArIF9rMi5Ub1N0cmluZygpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJSMlJvdCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICByZXR1cm47XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IoaywgX2syLCAxZS02KSlcclxuICAgIHtcclxuICAgICAgICBDb25zb2xlLldyaXRlTGluZShcInRlc3QzIGZhaWxlZFwiKTtcclxuICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKF9rMi5Ub1N0cmluZygpKTtcclxuICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlIyUm90IGZhaWxlZCB3aXRoIF9rMiA9IFwiICsgX2syLlRvU3RyaW5nKCkpO1xyXG4gICAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICBDb25zb2xlLldyaXRlTGluZShcIlIyUm90IHN1Y2NlZWRlZFwiKTtcclxufVxyXG5cclxuO1xuXHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxLjAsIDAsIDAgfSksIE1hdGguUEkgLyAyLjApO1xyXG4gICAgICAgICAgICBfUjJyb3RfdGVzdCh2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMS4wLCAwIH0pLCBNYXRoLlBJIC8gMi4wKTtcclxuICAgICAgICAgICAgX1Iycm90X3Rlc3Qodl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIDAsIDEuMCB9KSwgTWF0aC5QSSAvIDIuMCk7XHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjQ0OTAyMjEsIDAuMzAyMDc5NDUsIDAuODQwOTA4NTMgfSksIDIuNjU5NDk4ODQpO1xyXG5cclxuICAgICAgICAgICAgLy9TaW5ndWxhcml0aWVzXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDEuMCwgMi4wLCAzLjAgfSkgLyB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAyLjAsIDMuMCB9KS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgX1Iycm90X3Rlc3QoazEsIDFlLTEwKTtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsyID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDIuMCwgLTEuMCwgMy4wIH0pIC8gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDIuMCwgLTEuMCwgMy4wIH0pLkwyTm9ybSgpO1xyXG4gICAgICAgICAgICBfUjJyb3RfdGVzdChrMiwgTWF0aC5QSSArIDFlLTEwKTtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGszID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IC0yLjAsIC0xLjAsIDMuMCB9KSAvIHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAtMi4wLCAtMS4wLCAzLjAgfSkuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KGszLCBNYXRoLlBJICsgMWUtMTApO1xyXG5cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazQgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgLTIuMCwgLTEuMCwgMy4wIH0pIC8gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IC0yLjAsIC0xLjAsIDMuMCB9KS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgX1Iycm90X3Rlc3QoazQsIE1hdGguUEkgKyAxZS0xMCk7XHJcblxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBrNSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAtMS4wLCAtMy4wIH0pIC8gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIC0xLjAsIC0zLjAgfSkuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIF9SMnJvdF90ZXN0KGs1LCBNYXRoLlBJICsgMWUtMTApO1xyXG5cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazYgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMCwgMS4wIH0pO1xyXG4gICAgICAgICAgICBfUjJyb3RfdGVzdChrNiwgTWF0aC5QSSArIDFlLTEwKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHN0YXRpYyB2b2lkIFRlc3RTY3Jld01hdHJpeCgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlRlc3RpbmcgU2NyZXdNYXRyaXhcIik7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGsgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAyLjAsIDMuMCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gRyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU2NyZXdfbWF0cml4KGspO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBHX3QgPSBtX2J1aWxkZXIuRGVuc2VPZlJvd0FycmF5cyhcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMS4wLCAwLCAwLCAwLCAtMywgMiB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLCAxLjAsIDAsIDMsIDAsIC0xIH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAsIDAsIDEuMCwgLTIsIDEsIDAgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMCwgMCwgMCwgMS4wLCAwLCAwIH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAsIDAsIDAsIDAsIDEuMCwgMCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLCAwLCAwLCAwLCAwLCAxLjAgfSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzTWF0cml4KEcsIEdfdCwgMWUtOCkpIENvbnNvbGUuV3JpdGVMaW5lKFwiU2NyZXdNYXJpeCBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJTY3Jld01hdHJpeCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UjJRKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC41MDU3NjM5LCAtMC4xMzQwNTM3LCAwLjg1MjE5MjggfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC42NDU2OTYyLCAtMC43MTM5MjI0LCAwLjI3MDkwODEgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC41NzIwODMzLCAwLjY4NzI3MzEsIDAuNDQ3NjM0MiB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcV90ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuMjM4NzE5NCwgMC40MzYwNDAyLCAwLjI5MzM0NTksIDAuODE2NTk2NyB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUjJRKHJvdCk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHEsIHFfdCwgMWUtNikpIENvbnNvbGUuV3JpdGVMaW5lKFwiUjJRIGZhaWxlZFwiKTtcclxuICAgICAgICAgICAgZWxzZSBDb25zb2xlLldyaXRlTGluZShcIlIyUSBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UTJSKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdF90ID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMoXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IC0wLjUwNTc2MzksIC0wLjEzNDA1MzcsIDAuODUyMTkyOCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjY0NTY5NjIsIC0wLjcxMzkyMjQsIDAuMjcwOTA4MSB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjU3MjA4MzMsIDAuNjg3MjczMSwgMC40NDc2MzQyIH0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuMjM4NzE5NCwgMC40MzYwNDAyLCAwLjI5MzM0NTksIDAuODE2NTk2NyB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90ID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RMlIocSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzTWF0cml4KHJvdCwgcm90X3QsIDFlLTYpKSBDb25zb2xlLldyaXRlTGluZShcIlEyUiBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJRMlIgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgc3RhdGljIHZvaWQgVGVzdFJvdDJRKClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIFR1cGxlPFZlY3Rvcjxkb3VibGU+LCBkb3VibGU+IHJvdCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUjJyb3QobV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMoXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IC0wLjUwNTc2MzksIC0wLjEzNDA1MzcsIDAuODUyMTkyOCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjY0NTY5NjIsIC0wLjcxMzkyMjQsIDAuMjcwOTA4MSB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjU3MjA4MzMsIDAuNjg3MjczMSwgMC40NDc2MzQyIH0pKTtcclxuICAgICAgICAgICAgZG91YmxlW10gayA9IG5ldyBkb3VibGVbM107XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgcm90Lkl0ZW0xLkNvdW50OyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGtbaV0gPSAoZG91YmxlKXJvdC5JdGVtMVtpXTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBmbG9hdCB0aGV0YSA9IChmbG9hdClyb3QuSXRlbTI7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFfdCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjIzODcxOTQsIDAuNDM2MDQwMiwgMC4yOTMzNDU5LCAwLjgxNjU5NjcgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHEgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdDJRKGssIHRoZXRhKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IocSwgcV90LCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJSb3QyUSBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJSb3QyUSBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UTJSb3QoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcm90X3QgPSBtX2J1aWxkZXIuRGVuc2VPZlJvd0FycmF5cyhcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgLTAuNTA1NzYzOSwgLTAuMTM0MDUzNywgMC44NTIxOTI4IH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNjQ1Njk2MiwgLTAuNzEzOTIyNCwgMC4yNzA5MDgxIH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNTcyMDgzMywgMC42ODcyNzMxLCAwLjQ0NzYzNDIgfSk7XHJcbiAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUocm90X3QpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuMjM4NzE5NCwgMC40MzYwNDAyLCAwLjI5MzM0NTksIDAuODE2NTk2NyB9KTtcclxuICAgICAgICAgICAgVHVwbGU8VmVjdG9yPGRvdWJsZT4sIGRvdWJsZT4gcm90ID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RMlJvdChxKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gayA9IHJvdC5JdGVtMTtcclxuICAgICAgICAgICAgZG91YmxlIHRoZXRhID0gcm90Lkl0ZW0yO1xyXG4gICAgICAgICAgICAvL0NvbnNvbGUuV3JpdGVMaW5lKEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KGssIHRoZXRhKSk7XHJcbiAgICAgICAgICAgIC8vQ29uc29sZS5Xcml0ZUxpbmUocm90X3QpO1xyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFsc01hdHJpeChHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdChrLCB0aGV0YSksIHJvdF90LCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJRMlJvdCBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJRMlJvdCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UXVhdGNvbXBsZW1lbnQoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjIzODcxOTQsIDAuNDM2MDQwMiwgMC4yOTMzNDU5LCAwLjgxNjU5NjcgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFfYyA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUXVhdGNvbXBsZW1lbnQocSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzKHFbMF0sIHFfY1swXSwgMWUtOCkgfHxcclxuICAgICAgICAgICAgICAgICFBbG1vc3RFcXVhbHNWZWN0b3IocS5TdWJWZWN0b3IoMSwgMyksIC1xX2MuU3ViVmVjdG9yKDEsIDMpLCAxZS04KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJRdWF0Y29tcGxlbWVudCBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJRdWF0Y29tcGxlbWVudCBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UXVhdHByb2R1Y3QoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcV8xID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAuNjM4Njc4NzcsIDAuNTIyNTE3OTcsIDAuNTYxNTY1NzMsIDAuMDYwODk2MTUgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHFfMiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjM1NzY0NzE2LCAwLjYxMDUxNDI0LCAwLjExNTQwODAxLCAwLjY5NzE2NzAzIH0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBSX3QgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlEyUihxXzEpLk11bHRpcGx5KEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUTJSKHFfMikpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxX3QgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlIyUShSX3QpO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBxID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RdWF0cHJvZHVjdChxXzEpLk11bHRpcGx5KHFfMik7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHEsIHFfdCwgMWUtNikpIENvbnNvbGUuV3JpdGVMaW5lKFwiUXVhdHByb2R1Y3QgZmFpbGVkXCIpO1xyXG4gICAgICAgICAgICBlbHNlIENvbnNvbGUuV3JpdGVMaW5lKFwiUXVhdHByb2R1Y3Qgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgIH1cclxuXHJcblxyXG4gICAgICAgIHN0YXRpYyB2b2lkIFRlc3RRdWF0amFjb2JpYW4oKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjYzODY3ODc3LCAwLjUyMjUxNzk3LCAwLjU2MTU2NTczLCAwLjA2MDg5NjE1IH0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBKID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5RdWF0amFjb2JpYW4ocSk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IEpfdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKG5ld1tdIHsgLTAuMjYxMjU4OTgsIC0wLjI4MDc4Mjg2LCAtMC4wMzA0NDgwOCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjMxOTMzOTM4LCAwLjAzMDQ0ODA4LCAtMC4yODA3ODI4NiB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC4wMzA0NDgwOCwgMC4zMTkzMzkzOCwgMC4yNjEyNTg5OCB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjI4MDc4Mjg2LCAtMC4yNjEyNTg5OCwgMC4zMTkzMzkzOCB9KTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgoSiwgSl90LCAxZS02KSkgeyBDb25zb2xlLldyaXRlTGluZShcIlF1YXRqYWNvYmlhbiBmYWlsZWRcIik7IH1cclxuICAgICAgICAgICAgZWxzZSBDb25zb2xlLldyaXRlTGluZShcIlF1YXRqYWNvYmlhbiBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgdm9pZCBUZXN0UnB5MlIoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcnB5MSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxMCAqIE1hdGguUEkgLyAxODAsIC0zMCAqIE1hdGguUEkgLyAxODAsIDkwICogTWF0aC5QSSAvIDE4MCB9KTtcclxuXHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IFIxID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5ScHkyUihycHkxKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUjFfdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC4wMDAwMDAwLCAtMC45ODQ4MDc3LCAwLjE3MzY0ODIgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC44NjYwMjU0LCAtMC4wODY4MjQxLCAtMC40OTI0MDM5IH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNTAwMDAwMCwgMC4xNTAzODM3LCAwLjg1Mjg2ODYgfSk7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzTWF0cml4KFIxLCBSMV90LCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJScHkyUiBmYWlsZWRcIik7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHJweTIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlIyUnB5KFIxKTtcclxuICAgICAgICAgICAgLy9Db25zb2xlLldyaXRlTGluZShycHkyKTtcclxuICAgICAgICAgICAgaWYoIUFsbW9zdEVxdWFsc1ZlY3RvcihycHkxLCBycHkyLCAxZS02KSkgQ29uc29sZS5Xcml0ZUxpbmUoXCJScHkyUiBmYWlsZWRcIik7XHJcblxyXG4gICAgICAgICAgICAvLyBDaGVjayBzaW5ndWxhcml0eVxyXG4gICAgICAgICAgICAvLyBOT1RFOiBEb2VzIG5vdCByYWlzZSBleGNlcHRpb25cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcnB5MyA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxMCAqIE1hdGguUEkgLyAxODAsIDkwICogTWF0aC5QSSAvIDE4MCwgLTMwICogTWF0aC5QSSAvIDE4MCB9KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUjMgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJweTJSKHJweTMpO1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShHZW5lcmFsUm9ib3RpY3NUb29sYm94LlIyUnB5KFIzKS5Ub1N0cmluZygpKTtcclxuXHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiUnB5MlIgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgc3RhdGljIHZvaWQgVGVzdF9TdWJwcm9ibGVtcygpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB4ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDEuMCwgMCwgMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4geSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAxLjAsIDAgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHogPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMCwgMS4wIH0pO1xyXG5cclxuICAgICAgICAgICAgLy8gU3VicHJvYmxlbTBcclxuICAgICAgICAgICAgaWYgKEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTAoeCwgeSwgeikgPT0gTWF0aC5QSSAvIDIpIENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTAgc3VjY2VlZGVkXCIpO1xyXG4gICAgICAgICAgICBlbHNlIENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTAgZmFpbGVkXCIpO1xyXG5cclxuICAgICAgICAgICAgLy8gU3VicHJvYmxlbTFcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazEgPSAoeCArIHopIC8gKHggKyB6KS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gazIgPSAoeSArIHopIC8gKHkgKyB6KS5MMk5vcm0oKTtcclxuICAgICAgICAgICAgaWYgKEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTEoazEsIGsyLCB6KSA9PSBNYXRoLlBJIC8gMikgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMSBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgICAgIGVsc2UgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMSBmYWlsZWRcIik7XHJcblxyXG4gICAgICAgICAgICAvLyBTdWJwcm9ibGVtMlxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBwMiA9IHg7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHEyID0geC5BZGQoeSkuQWRkKHopO1xyXG4gICAgICAgICAgICBxMiA9IHEyIC8gcTIuTDJOb3JtKCk7XHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGEyID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5TdWJwcm9ibGVtMihwMiwgcTIsIHosIHkpO1xyXG4gICAgICAgICAgICBpZiAoYTIuTGVuZ3RoICE9IDQpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICAvL05PVEU6IERJRkZFUkVOVCBUSEFOIFBZVEhPTiBWRVJTSU9OXHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByMV8wID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTJbMF0pO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiByMV8xID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeSwgYTJbMV0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiByMSA9IChyMV8wICogcjFfMSkuQ29sdW1uKDApO1xyXG5cclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcjJfMCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHosIGEyWzJdKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcjJfMSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHksIGEyWzNdKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcjIgPSAocjJfMCAqIHIyXzEpLkNvbHVtbigwKTtcclxuXHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHIxLCBxMiwgMWUtNCkpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFsc1ZlY3RvcihyMiwgcTIsIDFlLTQpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTIgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuXHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGEzID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5TdWJwcm9ibGVtMih4LCB6LCB6LCB5KTtcclxuICAgICAgICAgICAgaWYgKGEzLkxlbmd0aCAhPSAyKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTIgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuICAgICAgICAgICAgLy9OT1RFOiBESUZGRVJFTlQgVEhBTiBQWVRIT04gVkVSU0lPTlxyXG5cclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcjNfMCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHosIGEzWzBdKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcjNfMSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHksIGEzWzFdKTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcjMgPSAocjNfMCAqIHIzXzEpLkNvbHVtbigwKTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IocjMsIHosIDFlLTQpKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTIgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMiBzdWNjZWVkZWRcIik7XHJcblxyXG4gICAgICAgICAgICAvLyBTdWJwcm9ibGVtM1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBwNCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAuNSwgMCwgMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcTQgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgLjc1LCAwIH0pO1xyXG5cclxuICAgICAgICAgICAgZG91YmxlW10gYTQgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlN1YnByb2JsZW0zKHA0LCBxNCwgeiwgLjUpO1xyXG4gICAgICAgICAgICBkb3VibGVbXSBhNSA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTMocDQsIHE0LCB6LCAxLjI1KTtcclxuICAgICAgICAgICAgaWYgKGE0Lkxlbmd0aCAhPSAyKSB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTMgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuXHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzKChxNCArIEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHosIGE0WzBdKSAqIHA0KS5MMk5vcm0oKSwgMC41LCAxZS04KSlcclxuICAgICAgICAgICAgeyBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0zIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzKChxNCArIEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHosIGE0WzFdKSAqIHA0KS5MMk5vcm0oKSwgMC41LCAxZS04KSlcclxuICAgICAgICAgICAgeyBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW0zIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcblxyXG4gICAgICAgICAgICBpZiAoYTUuTGVuZ3RoICE9IDEpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMyBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFscygocTQgKyBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdCh6LCBhNVswXSkgKiBwNCkuTDJOb3JtKCksIDEuMjUsIDFlLTgpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTMgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuICAgICAgICAgICAgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtMyBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgICAgIC8vIFN1YnByb2JsZW00XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHA2ID0geTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gcTYgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgLjgsIC4yLCAuNSB9KTtcclxuICAgICAgICAgICAgZG91YmxlIGQ2ID0gLjM7XHJcblxyXG4gICAgICAgICAgICBkb3VibGVbXSBhNiA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guU3VicHJvYmxlbTQocDYsIHE2LCB6LCBkNik7XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzKChwNiAqIEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHosIGE2WzBdKSAqIHE2KSwgZDYsIDFlLTQpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiU3VicHJvYmxlbTQgZmFpbGVkXCIpOyByZXR1cm47IH1cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHMoKHA2ICogR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Sb3QoeiwgYTZbMV0pICogcTYpLCBkNiwgMWUtNCkpXHJcbiAgICAgICAgICAgIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJTdWJwcm9ibGVtNCBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlN1YnByb2JsZW00IHN1Y2NlZWRlZFwiKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHN0YXRpYyBSb2JvdCBwdW1hMjYwYl9yb2JvdCgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICAvLyBSZXR1cm5zIGFuIGFwcHJveGltYXRlIFJvYm90IGluc3RhbmNlIGZvciBhIFB1bWEgMjYwQiByb2JvdFxyXG5cclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHggPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMS4wLCAwLCAwIH0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB5ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIDEuMCwgMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4geiA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAwLCAxLjAgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IGEgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wLCAwLCAwIH0pO1xyXG5cclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gSCA9IG1fYnVpbGRlci5EZW5zZU9mQ29sdW1uVmVjdG9ycyh6LCB5LCB5LCB6LCB5LCB4KTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gUCA9IDAuMDI1NCAqIG1fYnVpbGRlci5EZW5zZU9mQ29sdW1uVmVjdG9ycygxMyAqIHosIGEsICgtNC45ICogeSArIDcuOCAqIHggLSAwLjc1ICogeiksIC04LjAgKiB6LCBhLCBhLCAyLjIgKiB4KTtcclxuICAgICAgICAgICAgaW50W10gam9pbnRfdHlwZSA9IG5ld1tdIHsgMCwgMCwgMCwgMCwgMCwgMCB9O1xyXG4gICAgICAgICAgICBkb3VibGVbXSBqb2ludF9taW4gPSBuZXdbXSB7IC01LjAsIC0yNTYsIC0yMTQsIC0zODQsIC0zMiwgLTI2NyB9O1xyXG4gICAgICAgICAgICBkb3VibGVbXSBqb2ludF9tYXggPSBuZXdbXSB7IDMxMy4wLCA3NiwgMzQsIDE5NCwgMjEyLCAyNjcgfTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCBqb2ludF9taW4uTGVuZ3RoOyBpKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGpvaW50X21pbltpXSA9IGpvaW50X21pbltpXSAqIE1hdGguUEkgLyAxODAuMDtcclxuICAgICAgICAgICAgICAgIGpvaW50X21heFtpXSA9IGpvaW50X21heFtpXSAqIE1hdGguUEkgLyAxODAuMDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJvYm90KEgsIFAsIGpvaW50X3R5cGUsIGpvaW50X21pbiwgam9pbnRfbWF4KTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHN0YXRpYyBSb2JvdCBhYmJfaXJiNjY0MF8xODBfMjU1X3JvYm90KClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIC8vIFJldHVybnMgYW4gYXBwcm94aW1hdGUgUm9ib3QgaW5zdGFuY2UgZm9yIGEgUHVtYSAyNjBCIHJvYm90XHJcblxyXG5cclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4geCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAxLjAsIDAsIDAgfSk7XHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHkgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMS4wLCAwIH0pO1xyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiB6ID0gdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDAsIDAsIDEuMCB9KTtcclxuICAgICAgICAgICAgVmVjdG9yPGRvdWJsZT4gYSA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjAsIDAsIDAgfSk7XHJcblxyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBIID0gbV9idWlsZGVyLkRlbnNlT2ZDb2x1bW5WZWN0b3JzKHosIHksIHksIHgsIHksIHgpO1xyXG4gICAgICAgICAgICBNYXRyaXg8ZG91YmxlPiBQID0gbV9idWlsZGVyLkRlbnNlT2ZDb2x1bW5WZWN0b3JzKDAuNzggKiB6LCAwLjMyICogeCwgMS4wNzUgKiB6LCAwLjIgKiB6LCAxLjE0MiAqIHgsIDAuMiAqIHgsIGEpO1xyXG4gICAgICAgICAgICBpbnRbXSBqb2ludF90eXBlID0gbmV3W10geyAwLCAwLCAwLCAwLCAwLCAwIH07XHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGpvaW50X21pbiA9IG5ld1tdIHsgLTE3MC4wLCAtNjUsIC0xODAsIC0zMDAsIC0xMjAsIC0zNjAgfTtcclxuICAgICAgICAgICAgZG91YmxlW10gam9pbnRfbWF4ID0gbmV3W10geyAxNzAuMCwgODUsIDcwLCAzMDAsIDEyMCwgMzYwIH07XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgam9pbnRfbWluLkxlbmd0aDsgaSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBqb2ludF9taW5baV0gPSBqb2ludF9taW5baV0gKiBNYXRoLlBJIC8gMTgwLjA7XHJcbiAgICAgICAgICAgICAgICBqb2ludF9tYXhbaV0gPSBqb2ludF9tYXhbaV0gKiBNYXRoLlBJIC8gMTgwLjA7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBSb2JvdChILCBQLCBqb2ludF90eXBlLCBqb2ludF9taW4sIGpvaW50X21heCk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBzdGF0aWMgUm9ib3QgcHVtYTI2MGJfcm9ib3RfdG9vbCgpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBSb2JvdCByb2JvdCA9IHB1bWEyNjBiX3JvYm90KCk7XHJcbiAgICAgICAgICAgIHJvYm90LlJfdG9vbCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guUm90KHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLCAxLjAsIDAgfSksIE1hdGguUEkgLyAyLjApO1xyXG4gICAgICAgICAgICByb2JvdC5QX3Rvb2wgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4wNSwgMCwgMCB9KTtcclxuICAgICAgICAgICAgcmV0dXJuIHJvYm90O1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgc3RhdGljIHZvaWQgVGVzdEZ3ZGtpbigpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBSb2JvdCBwdW1hID0gcHVtYTI2MGJfcm9ib3QoKTtcclxuXHJcbiAgICAgICAgICAgIFRyYW5zZm9ybSBwb3NlID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Gd2RraW4ocHVtYSwgbmV3W10geyAwLjAsIDAuMCwgMC4wLCAwLjAsIDAuMCwgMC4wIH0pO1xyXG4gICAgICAgICAgICBpZighQWxtb3N0RXF1YWxzTWF0cml4KHBvc2UuUiwgbV9idWlsZGVyLkRlbnNlSWRlbnRpdHkoMyksIDFlLTgpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiRndkS2luIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcbiAgICAgICAgICAgIGlmICghQWxtb3N0RXF1YWxzVmVjdG9yKHBvc2UuUCwgdl9idWlsZGVyLkRlbnNlT2ZBcnJheShuZXdbXSB7IDEwLjAgKiBpbl8yX20sIC00LjkgKiBpbl8yX20sIDQuMjUgKiBpbl8yX20gfSksIDFlLTYpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiRndkS2luIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcblxyXG4gICAgICAgICAgICAvLyBBbm90aGVyIHJpZ2h0LWFuZ2xlIGNvbmZpZ3VyYXRpb25cclxuICAgICAgICAgICAgZG91YmxlW10gam9pbnRzMiA9IG5ld1tdIHsgMTgwLjAsIC05MCwgLTkwLCA5MCwgOTAsIDkwIH07XHJcbiAgICAgICAgICAgIGZvciAoaW50IGkgPSAwOyBpIDwgam9pbnRzMi5MZW5ndGg7IGkrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgam9pbnRzMltpXSA9IGpvaW50czJbaV0gKiBNYXRoLlBJIC8gMTgwLjA7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgVHJhbnNmb3JtIHBvc2UyID0gR2VuZXJhbFJvYm90aWNzVG9vbGJveC5Gd2RraW4ocHVtYSwgam9pbnRzMik7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHJvdDIgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdCh2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMCwgMS4wIH0pLCBNYXRoLlBJKS5NdWx0aXBseShHZW5lcmFsUm9ib3RpY3NUb29sYm94LlJvdCh2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMCwgMS4wLCAwIH0pLCAtTWF0aC5QSSAvIDIpKTtcclxuICAgICAgICAgICAgaWYoIUFsbW9zdEVxdWFsc01hdHJpeChwb3NlMi5SLCByb3QyLCAxZS02KSkgeyBDb25zb2xlLldyaXRlTGluZShcIkZ3ZEtpbiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBpZiAoIUFsbW9zdEVxdWFsc1ZlY3Rvcihwb3NlMi5QLCB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgLTAuNzUsIDQuOSwgMzEgfSkgKiAwLjAyNTQsIDFlLTYpKVxyXG4gICAgICAgICAgICB7IENvbnNvbGUuV3JpdGVMaW5lKFwiRndkS2luIGZhaWxlZFwiKTsgcmV0dXJuOyB9XHJcblxyXG4gICAgICAgICAgICAvL1JhbmRvbSBjb25maWd1cmF0aW9uXHJcbiAgICAgICAgICAgIGRvdWJsZVtdIGpvaW50czMgPSBuZXdbXSB7IDUwLjAsIC0xMDUsIDMxLCA0LCAxMjYsIC0xODQgfTtcclxuICAgICAgICAgICAgZm9yIChpbnQgaSA9IDA7IGkgPCBqb2ludHMzLkxlbmd0aDsgaSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBqb2ludHMzW2ldID0gam9pbnRzM1tpXSAqIE1hdGguUEkgLyAxODAuMDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBUcmFuc2Zvcm0gcG9zZTMgPSBHZW5lcmFsUm9ib3RpY3NUb29sYm94LkZ3ZGtpbihwdW1hLCBqb2ludHMzKTtcclxuICAgICAgICAgICAgTWF0cml4PGRvdWJsZT4gcG9zZTNfUl90ID0gbV9idWlsZGVyLkRlbnNlT2ZSb3dBcnJheXMoXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNDI3NCwgMC44MDY5LCAtMC40MDc2IH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNDQ1NSwgLTAuNTgwNCwgLTAuNjgxNyB9LFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAtMC43ODY2LCAwLjEwOTcsIC0wLjYwNzYgfSk7XHJcblxyXG4gICAgICAgICAgICBWZWN0b3I8ZG91YmxlPiBwb3NlM19QX3QgPSB2X2J1aWxkZXIuRGVuc2VPZkFycmF5KG5ld1tdIHsgMC4yMjM2LCAwLjA2OTMsIDAuNDI2NSB9KTtcclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNNYXRyaXgocG9zZTMuUiwgcG9zZTNfUl90LCAxZS00KSkgeyBDb25zb2xlLldyaXRlTGluZShcIkZ3ZEtpbiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBpZighQWxtb3N0RXF1YWxzVmVjdG9yKHBvc2UzLlAsIHBvc2UzX1BfdCwgMWUtNCkpIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJGd2RLaW4gZmFpbGVkXCIpOyByZXR1cm47IH1cclxuXHJcbiAgICAgICAgICAgIFJvYm90IHB1bWFfdG9vbCA9IHB1bWEyNjBiX3JvYm90X3Rvb2woKTtcclxuXHJcbiAgICAgICAgICAgIFRyYW5zZm9ybSBwb3NlNCA9IEdlbmVyYWxSb2JvdGljc1Rvb2xib3guRndka2luKHB1bWFfdG9vbCwgam9pbnRzMyk7XHJcbiAgICAgICAgICAgIE1hdHJpeDxkb3VibGU+IHBvc2U0X1JfdCA9IG1fYnVpbGRlci5EZW5zZU9mUm93QXJyYXlzKFxyXG4gICAgICAgICAgICAgICAgbmV3W10geyAwLjQwNzYsIDAuODA2OSwgMC40Mjc0IH0sXHJcbiAgICAgICAgICAgICAgICBuZXdbXSB7IDAuNjgxNjU0LCAtMC41ODAzNTcsIDAuNDQ1NTcgfSxcclxuICAgICAgICAgICAgICAgIG5ld1tdIHsgMC42MDc1OSwgMC4xMDk3LCAtMC43ODY2IH0pO1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlJvYm90IFIgdG9vbD17MH1cIiwgcG9zZTQuUik7XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiUm9ib3QgUiBjYWxjdWxhdGVkIHRvb2w9ezB9XCIsIHBvc2U0X1JfdCk7XHJcbiAgICAgICAgICAgIENvbnNvbGUuV3JpdGVMaW5lKFwiUm9ib3QgcCB0b29sPXswfVwiLCBwb3NlNC5QKTtcclxuXHJcbiAgICAgICAgICAgIFZlY3Rvcjxkb3VibGU+IHBvc2U0X1BfdCA9IHZfYnVpbGRlci5EZW5zZU9mQXJyYXkobmV3W10geyAwLjI0NTAsIDAuMDkxNiwgMC4zODcyIH0pO1xyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIlJvYm90IHAgY2FsY3VsYXRlZCB0b29sPXswfVwiLCBwb3NlNF9QX3QpO1xyXG4gICAgICAgICAgICBpZighQWxtb3N0RXF1YWxzTWF0cml4KHBvc2U0LlIsIHBvc2U0X1JfdCwgMSAqIDEwIF4gNCkpXHJcbiAgICAgICAgICAgIHsgQ29uc29sZS5Xcml0ZUxpbmUoXCJGd2RLaW4gZmFpbGVkXCIpOyByZXR1cm47IH1cclxuICAgICAgICAgICAgaWYgKCFBbG1vc3RFcXVhbHNWZWN0b3IocG9zZTQuUCwgcG9zZTRfUF90LCAxICogMTAgXiA0KSlcclxuICAgICAgICAgICAgeyBDb25zb2xlLldyaXRlTGluZShcIkZ3ZEtpbiBmYWlsZWRcIik7IHJldHVybjsgfVxyXG4gICAgICAgICAgICBDb25zb2xlLldyaXRlTGluZShcIkZ3ZEtpbiBzdWNjZWVkZWRcIik7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBwdWJsaWMgc3RhdGljIHZvaWQgUnVuVGVzdHMoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgLy9UZXN0SGF0KCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFJvdCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RSMlJvdCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RTY3Jld01hdHJpeCgpO1xyXG4gICAgICAgICAgICAvL1Rlc3RSMlEoKTtcclxuICAgICAgICAgICAgLy9UZXN0UTJSKCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFJvdDJRKCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFEyUm90KCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFF1YXRjb21wbGVtZW50KCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFF1YXRwcm9kdWN0KCk7XHJcbiAgICAgICAgICAgIC8vVGVzdFF1YXRqYWNvYmlhbigpO1xyXG4gICAgICAgICAgICBUZXN0UnB5MlIoKTtcclxuICAgICAgICAgICAgVGVzdF9TdWJwcm9ibGVtcygpO1xyXG4gICAgICAgICAgICBUZXN0Rndka2luKCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59XHJcbiJdCn0K
