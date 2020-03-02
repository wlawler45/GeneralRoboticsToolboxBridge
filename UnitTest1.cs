using System;
using System.Collections.Generic;
//using Microsoft.VisualStudio.TestTools.UnitTesting;
using TestGeneralRoboticsToolboxNET;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Double.MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;
using Bridge;
using Bridge.Html5;
using Bridge.QUnit;

namespace GeneralRoboticsToolboxTests
{
    [FileName("../samples/qunit/output/test.html")]
    public class UnitTest1
    {
        static double in_2_m = 0.0254;

        static VectorBuilder<double> v_builder = BuilderInstance<double>.Vector;
        static MatrixBuilder<double> m_builder = BuilderInstance<double>.Matrix;
        public static bool AlmostEquals(double val1, double val2, double accuracy = 1e-8)
        {
            return Math.Abs(val1 - val2) < accuracy;
        }
        public static bool AlmostEqualsMatrix(Matrix<double> val1, Matrix<double> val2, double accuracy = 1e-8)
        {
            if (val1.ColumnCount != val2.ColumnCount || val1.RowCount != val2.RowCount) return false;
            for (int i = 0; i < val1.RowCount; i++)
            {
                for (int j = 0; j < val1.ColumnCount; j++)
                {
                    if (Math.Abs(val1[i, j] - val2[i, j]) > accuracy) {
                        
                        return false;
                    }
                }
            }

            return true;
        }

        public static bool AlmostEqualsVector(Vector<double> val1, Vector<double> val2, double accuracy = 1e-8)
        {
            if (val1.Count != val2.Count) return false;
            for (int i = 0; i < val1.Count; i++)
            {
                if (Math.Abs(val1[i] - val2[i]) > accuracy) return false;
            }
            return true;
        }

        static void TestHat()
        {
            //Test Hat
            Console.WriteLine("Testing Hat...");
            Vector<double> k = v_builder.DenseOfArray(new[] { 1.0, 2.0, 3.0 });
            Matrix<double> khat = m_builder.Dense(3, 3);
            khat[0, 1] = -3;
            khat[0, 2] = 2;
            khat[1, 0] = 3;
            khat[1, 2] = -1;
            khat[2, 0] = -2;
            khat[2, 1] = 1;
            Matrix<double> k_hat = GeneralRoboticsToolbox.Hat(k);
            if (!AlmostEqualsMatrix(k_hat, khat)) { Console.WriteLine("hat failed"); }
            else { Console.WriteLine("hat succeeded"); }
        }

        static void TestRot()
        {
            Vector<double> k = v_builder.DenseOfArray(new[] { 1.0, 0.0, 0.0 });
            Matrix<double> rot1 = m_builder.Dense(3, 3);
            rot1[0, 0] = 1;
            rot1[0, 1] = 0;
            rot1[0, 2] = 0;
            rot1[1, 0] = 0;
            rot1[1, 1] = 0;
            rot1[1, 2] = 1;
            rot1[2, 0] = 0;
            rot1[2, 1] = -1;
            rot1[2, 2] = 0;
            rot1 = rot1.Transpose();
            Console.WriteLine("Testing Rot...");
            Matrix<double> rot = GeneralRoboticsToolbox.Rot(k, Math.PI / 2.0);
            //Console.WriteLine(k.ToString());
            //Console.WriteLine(rot1.ToString());
            //Console.WriteLine(rot.ToString());

            if (!AlmostEqualsMatrix(rot1, rot, 1e-6)) { Console.WriteLine("rot1 failed"); }
            else { Console.WriteLine("rot1 succeeded"); }

            Matrix<double> rot2 = m_builder.DenseOfRowArrays(new[] { 0, 0, -1.0 }, new[] { 0, 1.0, 0 }, new[] { 1.0, 0, 0 }).Transpose();
            Vector<double> k2 = v_builder.DenseOfArray(new[] { 0, 1.0, 0 });
            Matrix<double> rot_2 = GeneralRoboticsToolbox.Rot(k2, Math.PI / 2);
            if (!AlmostEqualsMatrix(rot2, rot_2, 1e-6)) { Console.WriteLine("rot2 failed"); }
            else { Console.WriteLine("rot2 succeeded"); }


            Matrix<double> rot3 = m_builder.DenseOfRowArrays(new[] { 0, 1.0, 0 }, new[] { -1.0, 0, 0 }, new[] { 0, 0, 1.0 }).Transpose();
            Vector<double> k3 = v_builder.DenseOfArray(new[] { 0, 0, 1.0 });
            Matrix<double> rot_3 = GeneralRoboticsToolbox.Rot(k3, Math.PI / 2);
            if (!AlmostEqualsMatrix(rot3, rot_3, 1e-6)) { Console.WriteLine("rot3 failed"); }
            else { Console.WriteLine("rot3 succeeded"); }

            Matrix<double> rot4 = m_builder.DenseOfRowArrays(new[] { -0.5057639, -0.1340537, 0.8521928 }, new[] { 0.6456962, -0.7139224, 0.2709081 }, new[] { 0.5720833, 0.6872731, 0.4476342 });
            Vector<double> k4 = v_builder.DenseOfArray(new[] { 0.4490221, 0.30207945, 0.84090853 });
            Matrix<double> rot_4 = GeneralRoboticsToolbox.Rot(k4, 2.65949884);
            if (!AlmostEqualsMatrix(rot4, rot_4, 1e-5)) { Console.WriteLine("rot4 failed"); }
            else { Console.WriteLine("rot4 succeeded"); }
        }

        static void TestR2Rot()
        {
            void _R2rot_test(Vector<double> k, double theta1)
            {
                Matrix<double> R = GeneralRoboticsToolbox.Rot(k, theta1);
                Tuple<Vector<double>, double> r2_vals = GeneralRoboticsToolbox.R2rot(R);
                Vector<double> _k2 = r2_vals.Item1;
                double theta2 = r2_vals.Item2;

                if (Math.Abs(theta1 - theta2) > (theta1 + theta2))
                {
                    _k2 = -_k2;
                    theta2 = -theta2;
                }
                if (!AlmostEquals(theta1, theta2, 1e-6)) { Console.WriteLine("R2Rot failed with theta = " + theta1); }
                if (Math.Abs(theta1) < 1e-9)
                {
                    Console.WriteLine("R2Rot succeeded");
                    return;
                }
                if ((Math.Abs(theta1) - Math.PI) < 1e-9)
                {
                    if ((k + _k2).L2Norm() < 1e-6)
                    {
                        if (!AlmostEqualsVector(k, -_k2, 1e-6)) {
                            Console.WriteLine("test1 failed");
                            //Console.WriteLine((-_k2).ToString());
                            Console.WriteLine("R2Rot failed with -_k2 = " + (-_k2).ToString());
                        } else 
                        {
                            Console.WriteLine("R2Rot succeeded");
                        }
                        return;
                    }
                    if (!AlmostEqualsVector(k, _k2, 1e-6)) {
                        Console.WriteLine("test2 failed");
                        //Console.WriteLine(_k2.ToString());
                        Console.WriteLine("R2Rot failed with _k2 = " + _k2.ToString());
                    } else
                    {
                        Console.WriteLine("R2Rot succeeded");
                    }
                    return;
                }   
                if (!AlmostEqualsVector(k, _k2, 1e-6)) {
                    Console.WriteLine("test3 failed");
                    //Console.WriteLine(_k2.ToString());
                    Console.WriteLine("R2Rot failed with _k2 = " + _k2.ToString());
                    return;
                }
                Console.WriteLine("R2Rot succeeded");
            }

            _R2rot_test(v_builder.DenseOfArray(new[] { 1.0, 0, 0 }), Math.PI / 2.0);
            _R2rot_test(v_builder.DenseOfArray(new[] { 0, 1.0, 0 }), Math.PI / 2.0);
            _R2rot_test(v_builder.DenseOfArray(new[] { 0, 0, 1.0 }), Math.PI / 2.0);
            _R2rot_test(v_builder.DenseOfArray(new[] { 0.4490221, 0.30207945, 0.84090853 }), 2.65949884);

            //Singularities
            Vector<double> k1 = v_builder.DenseOfArray(new[] { 1.0, 2.0, 3.0 }) / v_builder.DenseOfArray(new[] { 1.0, 2.0, 3.0 }).L2Norm();
            _R2rot_test(k1, 1e-10);

            Vector<double> k2 = v_builder.DenseOfArray(new[] { 2.0, -1.0, 3.0 }) / v_builder.DenseOfArray(new[] { 2.0, -1.0, 3.0 }).L2Norm();
            _R2rot_test(k2, Math.PI + 1e-10);

            Vector<double> k3 = v_builder.DenseOfArray(new[] { -2.0, -1.0, 3.0 }) / v_builder.DenseOfArray(new[] { -2.0, -1.0, 3.0 }).L2Norm();
            _R2rot_test(k3, Math.PI + 1e-10);

            Vector<double> k4 = v_builder.DenseOfArray(new[] { -2.0, -1.0, 3.0 }) / v_builder.DenseOfArray(new[] { -2.0, -1.0, 3.0 }).L2Norm();
            _R2rot_test(k4, Math.PI + 1e-10);

            Vector<double> k5 = v_builder.DenseOfArray(new[] { 0, -1.0, -3.0 }) / v_builder.DenseOfArray(new[] { 0, -1.0, -3.0 }).L2Norm();
            _R2rot_test(k5, Math.PI + 1e-10);

            Vector<double> k6 = v_builder.DenseOfArray(new[] { 0, 0, 1.0 });
            _R2rot_test(k6, Math.PI + 1e-10);
        }

        static void TestScrewMatrix()
        {
            Console.WriteLine("Testing ScrewMatrix");
            Vector<double> k = v_builder.DenseOfArray(new[] { 1.0, 2.0, 3.0 });
            Matrix<double> G = GeneralRoboticsToolbox.Screw_matrix(k);
            Matrix<double> G_t = m_builder.DenseOfRowArrays(
                new[] { 1.0, 0, 0, 0, -3, 2 },
                new[] { 0, 1.0, 0, 3, 0, -1 },
                new[] { 0, 0, 1.0, -2, 1, 0 },
                new[] { 0, 0, 0, 1.0, 0, 0 },
                new[] { 0, 0, 0, 0, 1.0, 0 },
                new[] { 0, 0, 0, 0, 0, 1.0 });
            if (!AlmostEqualsMatrix(G, G_t, 1e-8)) Console.WriteLine("ScrewMarix failed");
            else Console.WriteLine("ScrewMatrix succeeded");
        }

        static void TestR2Q()
        {
            Matrix<double> rot = m_builder.DenseOfRowArrays(
                new[] { -0.5057639, -0.1340537, 0.8521928 },
                new[] { 0.6456962, -0.7139224, 0.2709081 },
                new[] { 0.5720833, 0.6872731, 0.4476342 });
            Vector<double> q_t = v_builder.DenseOfArray(new[] { 0.2387194, 0.4360402, 0.2933459, 0.8165967 });
            Vector<double> q = GeneralRoboticsToolbox.R2Q(rot);
            if (!AlmostEqualsVector(q, q_t, 1e-6)) Console.WriteLine("R2Q failed");
            else Console.WriteLine("R2Q succeeded");
        }

        static void TestQ2R()
        {
            Matrix<double> rot_t = m_builder.DenseOfRowArrays(
                new[] { -0.5057639, -0.1340537, 0.8521928 },
                new[] { 0.6456962, -0.7139224, 0.2709081 },
                new[] { 0.5720833, 0.6872731, 0.4476342 });
            Vector<double> q = v_builder.DenseOfArray(new[] { 0.2387194, 0.4360402, 0.2933459, 0.8165967 });
            Matrix<double> rot = GeneralRoboticsToolbox.Q2R(q);
            if (!AlmostEqualsMatrix(rot, rot_t, 1e-6)) Console.WriteLine("Q2R failed");
            else Console.WriteLine("Q2R succeeded");
        }

        static void TestRot2Q()
        {
            Tuple<Vector<double>, double> rot = GeneralRoboticsToolbox.R2rot(m_builder.DenseOfRowArrays(
                new[] { -0.5057639, -0.1340537, 0.8521928 },
                new[] { 0.6456962, -0.7139224, 0.2709081 },
                new[] { 0.5720833, 0.6872731, 0.4476342 }));
            double[] k = new double[3];
            for (int i = 0; i < rot.Item1.Count; i++)
            {
                k[i] = (double)rot.Item1[i];
            }
            float theta = (float)rot.Item2;
            Vector<double> q_t = v_builder.DenseOfArray(new[] { 0.2387194, 0.4360402, 0.2933459, 0.8165967 });
            Vector<double> q = GeneralRoboticsToolbox.Rot2Q(k, theta);
            if (!AlmostEqualsVector(q, q_t, 1e-6)) Console.WriteLine("Rot2Q failed");
            else Console.WriteLine("Rot2Q succeeded");
        }

        static void TestQ2Rot()
        {
            Matrix<double> rot_t = m_builder.DenseOfRowArrays(
                new[] { -0.5057639, -0.1340537, 0.8521928 },
                new[] { 0.6456962, -0.7139224, 0.2709081 },
                new[] { 0.5720833, 0.6872731, 0.4476342 });
            //Console.WriteLine(rot_t);
            Vector<double> q = v_builder.DenseOfArray(new[] { 0.2387194, 0.4360402, 0.2933459, 0.8165967 });
            Tuple<Vector<double>, double> rot = GeneralRoboticsToolbox.Q2Rot(q);
            Vector<double> k = rot.Item1;
            double theta = rot.Item2;
            //Console.WriteLine(GeneralRoboticsToolbox.Rot(k, theta));
            //Console.WriteLine(rot_t);
            if (!AlmostEqualsMatrix(GeneralRoboticsToolbox.Rot(k, theta), rot_t, 1e-6)) Console.WriteLine("Q2Rot failed");
            else Console.WriteLine("Q2Rot succeeded");
        }

        static void TestQuatcomplement()
        {
            Vector<double> q = v_builder.DenseOfArray(new[] { 0.2387194, 0.4360402, 0.2933459, 0.8165967 });
            Vector<double> q_c = GeneralRoboticsToolbox.Quatcomplement(q);
            if (!AlmostEquals(q[0], q_c[0], 1e-8) ||
                !AlmostEqualsVector(q.SubVector(1, 3), -q_c.SubVector(1, 3), 1e-8)) Console.WriteLine("Quatcomplement failed");
            else Console.WriteLine("Quatcomplement succeeded");
        }

        static void TestQuatproduct()
        {
            Vector<double> q_1 = v_builder.DenseOfArray(new[] { 0.63867877, 0.52251797, 0.56156573, 0.06089615 });
            Vector<double> q_2 = v_builder.DenseOfArray(new[] { 0.35764716, 0.61051424, 0.11540801, 0.69716703 });
            Matrix<double> R_t = GeneralRoboticsToolbox.Q2R(q_1).Multiply(GeneralRoboticsToolbox.Q2R(q_2));
            Vector<double> q_t = GeneralRoboticsToolbox.R2Q(R_t);
            Vector<double> q = GeneralRoboticsToolbox.Quatproduct(q_1).Multiply(q_2);
            if (!AlmostEqualsVector(q, q_t, 1e-6)) Console.WriteLine("Quatproduct failed");
            else Console.WriteLine("Quatproduct succeeded");
        }


        static void TestQuatjacobian()
        {
            Vector<double> q = v_builder.DenseOfArray(new[] { 0.63867877, 0.52251797, 0.56156573, 0.06089615 });
            Matrix<double> J = GeneralRoboticsToolbox.Quatjacobian(q);
            Matrix<double> J_t = m_builder.DenseOfRowArrays(new[] { -0.26125898, -0.28078286, -0.03044808 },
                new[] { 0.31933938, 0.03044808, -0.28078286 },
                new[] { -0.03044808, 0.31933938, 0.26125898 },
                new[] { 0.28078286, -0.26125898, 0.31933938 });
            if (!AlmostEqualsMatrix(J, J_t, 1e-6)) { Console.WriteLine("Quatjacobian failed"); }
            else Console.WriteLine("Quatjacobian succeeded");
        }

        static void TestRpy2R()
        {
            Vector<double> rpy1 = v_builder.DenseOfArray(new[] { 10 * Math.PI / 180, -30 * Math.PI / 180, 90 * Math.PI / 180 });

            Matrix<double> R1 = GeneralRoboticsToolbox.Rpy2R(rpy1);
            Matrix<double> R1_t = m_builder.DenseOfRowArrays(
                new[] { -0.0000000, -0.9848077, 0.1736482 },
                new[] { 0.8660254, -0.0868241, -0.4924039 },
                new[] { 0.5000000, 0.1503837, 0.8528686 });
            if (!AlmostEqualsMatrix(R1, R1_t, 1e-6)) Console.WriteLine("Rpy2R failed");
            Vector<double> rpy2 = GeneralRoboticsToolbox.R2Rpy(R1);
            //Console.WriteLine(rpy2);
            if(!AlmostEqualsVector(rpy1, rpy2, 1e-6)) Console.WriteLine("Rpy2R failed");

            // Check singularity
            // NOTE: Does not raise exception
            Vector<double> rpy3 = v_builder.DenseOfArray(new[] { 10 * Math.PI / 180, 90 * Math.PI / 180, -30 * Math.PI / 180 });
            Matrix<double> R3 = GeneralRoboticsToolbox.Rpy2R(rpy3);
            Console.WriteLine(GeneralRoboticsToolbox.R2Rpy(R3).ToString());

            Console.WriteLine("Rpy2R succeeded");
        }

        static void Test_Subproblems()
        {
            Vector<double> x = v_builder.DenseOfArray(new[] { 1.0, 0, 0 });
            Vector<double> y = v_builder.DenseOfArray(new[] { 0, 1.0, 0 });
            Vector<double> z = v_builder.DenseOfArray(new[] { 0, 0, 1.0 });

            // Subproblem0
            if (GeneralRoboticsToolbox.Subproblem0(x, y, z) == Math.PI / 2) Console.WriteLine("Subproblem0 succeeded");
            else Console.WriteLine("Subproblem0 failed");

            // Subproblem1
            Vector<double> k1 = (x + z) / (x + z).L2Norm();
            Vector<double> k2 = (y + z) / (y + z).L2Norm();
            if (GeneralRoboticsToolbox.Subproblem1(k1, k2, z) == Math.PI / 2) Console.WriteLine("Subproblem1 succeeded");
            else Console.WriteLine("Subproblem1 failed");

            // Subproblem2
            Vector<double> p2 = x;
            Vector<double> q2 = x.Add(y).Add(z);
            q2 = q2 / q2.L2Norm();
            double[] a2 = GeneralRoboticsToolbox.Subproblem2(p2, q2, z, y);
            if (a2.Length != 4) { Console.WriteLine("Subproblem2 failed"); return; }
            //NOTE: DIFFERENT THAN PYTHON VERSION

            Matrix<double> r1_0 = GeneralRoboticsToolbox.Rot(z, a2[0]);
            Matrix<double> r1_1 = GeneralRoboticsToolbox.Rot(y, a2[1]);
            Vector<double> r1 = (r1_0 * r1_1).Column(0);

            Matrix<double> r2_0 = GeneralRoboticsToolbox.Rot(z, a2[2]);
            Matrix<double> r2_1 = GeneralRoboticsToolbox.Rot(y, a2[3]);
            Vector<double> r2 = (r2_0 * r2_1).Column(0);

            if (!AlmostEqualsVector(r1, q2, 1e-4)) { Console.WriteLine("Subproblem2 failed"); return; }
            if (!AlmostEqualsVector(r2, q2, 1e-4)) { Console.WriteLine("Subproblem2 failed"); return; }

            double[] a3 = GeneralRoboticsToolbox.Subproblem2(x, z, z, y);
            if (a3.Length != 2) { Console.WriteLine("Subproblem2 failed"); return; }
            //NOTE: DIFFERENT THAN PYTHON VERSION

            Matrix<double> r3_0 = GeneralRoboticsToolbox.Rot(z, a3[0]);
            Matrix<double> r3_1 = GeneralRoboticsToolbox.Rot(y, a3[1]);
            Vector<double> r3 = (r3_0 * r3_1).Column(0);
            if (!AlmostEqualsVector(r3, z, 1e-4)) { Console.WriteLine("Subproblem2 failed"); return; }
            Console.WriteLine("Subproblem2 succeeded");

            // Subproblem3
            Vector<double> p4 = v_builder.DenseOfArray(new[] { .5, 0, 0 });
            Vector<double> q4 = v_builder.DenseOfArray(new[] { 0, .75, 0 });

            double[] a4 = GeneralRoboticsToolbox.Subproblem3(p4, q4, z, .5);
            double[] a5 = GeneralRoboticsToolbox.Subproblem3(p4, q4, z, 1.25);
            if (a4.Length != 2) { Console.WriteLine("Subproblem3 failed"); return; }

            if (!AlmostEquals((q4 + GeneralRoboticsToolbox.Rot(z, a4[0]) * p4).L2Norm(), 0.5, 1e-8))
            { Console.WriteLine("Subproblem3 failed"); return; }
            if (!AlmostEquals((q4 + GeneralRoboticsToolbox.Rot(z, a4[1]) * p4).L2Norm(), 0.5, 1e-8))
            { Console.WriteLine("Subproblem3 failed"); return; }

            if (a5.Length != 1) { Console.WriteLine("Subproblem3 failed"); return; }
            if (!AlmostEquals((q4 + GeneralRoboticsToolbox.Rot(z, a5[0]) * p4).L2Norm(), 1.25, 1e-8))
            { Console.WriteLine("Subproblem3 failed"); return; }
            Console.WriteLine("Subproblem3 succeeded");
            // Subproblem4
            Vector<double> p6 = y;
            Vector<double> q6 = v_builder.DenseOfArray(new[] { .8, .2, .5 });
            double d6 = .3;

            double[] a6 = GeneralRoboticsToolbox.Subproblem4(p6, q6, z, d6);
            if (!AlmostEquals((p6 * GeneralRoboticsToolbox.Rot(z, a6[0]) * q6), d6, 1e-4))
            { Console.WriteLine("Subproblem4 failed"); return; }
            if (!AlmostEquals((p6 * GeneralRoboticsToolbox.Rot(z, a6[1]) * q6), d6, 1e-4))
            { Console.WriteLine("Subproblem4 failed"); return; }
            Console.WriteLine("Subproblem4 succeeded");
        }

        static Robot puma260b_robot()
        {
            // Returns an approximate Robot instance for a Puma 260B robot


            Vector<double> x = v_builder.DenseOfArray(new[] { 1.0, 0, 0 });
            Vector<double> y = v_builder.DenseOfArray(new[] { 0, 1.0, 0 });
            Vector<double> z = v_builder.DenseOfArray(new[] { 0, 0, 1.0 });
            Vector<double> a = v_builder.DenseOfArray(new[] { 0.0, 0, 0 });

            Matrix<double> H = m_builder.DenseOfColumnVectors(z, y, y, z, y, x);
            Matrix<double> P = 0.0254 * m_builder.DenseOfColumnVectors(13 * z, a, (-4.9 * y + 7.8 * x - 0.75 * z), -8.0 * z, a, a, 2.2 * x);
            int[] joint_type = new[] { 0, 0, 0, 0, 0, 0 };
            double[] joint_min = new[] { -5.0, -256, -214, -384, -32, -267 };
            double[] joint_max = new[] { 313.0, 76, 34, 194, 212, 267 };
            for (int i = 0; i < joint_min.Length; i++)
            {
                joint_min[i] = joint_min[i] * Math.PI / 180.0;
                joint_max[i] = joint_max[i] * Math.PI / 180.0;
            }
            return new Robot(H, P, joint_type, joint_min, joint_max);
        }

        static Robot abb_irb6640_180_255_robot()
        {
            // Returns an approximate Robot instance for a Puma 260B robot


            Vector<double> x = v_builder.DenseOfArray(new[] { 1.0, 0, 0 });
            Vector<double> y = v_builder.DenseOfArray(new[] { 0, 1.0, 0 });
            Vector<double> z = v_builder.DenseOfArray(new[] { 0, 0, 1.0 });
            Vector<double> a = v_builder.DenseOfArray(new[] { 0.0, 0, 0 });

            Matrix<double> H = m_builder.DenseOfColumnVectors(z, y, y, x, y, x);
            Matrix<double> P = m_builder.DenseOfColumnVectors(0.78 * z, 0.32 * x, 1.075 * z, 0.2 * z, 1.142 * x, 0.2 * x, a);
            int[] joint_type = new[] { 0, 0, 0, 0, 0, 0 };
            double[] joint_min = new[] { -170.0, -65, -180, -300, -120, -360 };
            double[] joint_max = new[] { 170.0, 85, 70, 300, 120, 360 };
            for (int i = 0; i < joint_min.Length; i++)
            {
                joint_min[i] = joint_min[i] * Math.PI / 180.0;
                joint_max[i] = joint_max[i] * Math.PI / 180.0;
            }
            return new Robot(H, P, joint_type, joint_min, joint_max);
        }

        static Robot puma260b_robot_tool()
        {
            Robot robot = puma260b_robot();
            robot.R_tool = GeneralRoboticsToolbox.Rot(v_builder.DenseOfArray(new[] { 0, 1.0, 0 }), Math.PI / 2.0);
            robot.P_tool = v_builder.DenseOfArray(new[] { 0.05, 0, 0 });
            return robot;
        }

        static void TestFwdkin()
        {
            Robot puma = puma260b_robot();

            Transform pose = GeneralRoboticsToolbox.Fwdkin(puma, new[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
            if(!AlmostEqualsMatrix(pose.R, m_builder.DenseIdentity(3), 1e-8))
            { Console.WriteLine("FwdKin failed"); return; }
            if (!AlmostEqualsVector(pose.P, v_builder.DenseOfArray(new[] { 10.0 * in_2_m, -4.9 * in_2_m, 4.25 * in_2_m }), 1e-6))
            { Console.WriteLine("FwdKin failed"); return; }

            // Another right-angle configuration
            double[] joints2 = new[] { 180.0, -90, -90, 90, 90, 90 };
            for (int i = 0; i < joints2.Length; i++)
            {
                joints2[i] = joints2[i] * Math.PI / 180.0;
            }
            Transform pose2 = GeneralRoboticsToolbox.Fwdkin(puma, joints2);
            Matrix<double> rot2 = GeneralRoboticsToolbox.Rot(v_builder.DenseOfArray(new[] { 0, 0, 1.0 }), Math.PI).Multiply(GeneralRoboticsToolbox.Rot(v_builder.DenseOfArray(new[] { 0, 1.0, 0 }), -Math.PI / 2));
            if(!AlmostEqualsMatrix(pose2.R, rot2, 1e-6)) { Console.WriteLine("FwdKin failed"); return; }
            if (!AlmostEqualsVector(pose2.P, v_builder.DenseOfArray(new[] { -0.75, 4.9, 31 }) * 0.0254, 1e-6))
            { Console.WriteLine("FwdKin failed"); return; }

            //Random configuration
            double[] joints3 = new[] { 50.0, -105, 31, 4, 126, -184 };
            for (int i = 0; i < joints3.Length; i++)
            {
                joints3[i] = joints3[i] * Math.PI / 180.0;
            }
            Transform pose3 = GeneralRoboticsToolbox.Fwdkin(puma, joints3);
            Matrix<double> pose3_R_t = m_builder.DenseOfRowArrays(
                new[] { 0.4274, 0.8069, -0.4076 },
                new[] { 0.4455, -0.5804, -0.6817 },
                new[] { -0.7866, 0.1097, -0.6076 });

            Vector<double> pose3_P_t = v_builder.DenseOfArray(new[] { 0.2236, 0.0693, 0.4265 });
            if (!AlmostEqualsMatrix(pose3.R, pose3_R_t, 1e-4)) { Console.WriteLine("FwdKin failed"); return; }
            if(!AlmostEqualsVector(pose3.P, pose3_P_t, 1e-4)) { Console.WriteLine("FwdKin failed"); return; }

            Robot puma_tool = puma260b_robot_tool();

            Transform pose4 = GeneralRoboticsToolbox.Fwdkin(puma_tool, joints3);
            Matrix<double> pose4_R_t = m_builder.DenseOfRowArrays(
                new[] { 0.4076, 0.8069, 0.4274 },
                new[] { 0.681654, -0.580357, 0.44557 },
                new[] { 0.60759, 0.1097, -0.7866 });
            Console.WriteLine("Robot R tool={0}", pose4.R);
            Console.WriteLine("Robot R calculated tool={0}", pose4_R_t);
            Console.WriteLine("Robot p tool={0}", pose4.P);

            Vector<double> pose4_P_t = v_builder.DenseOfArray(new[] { 0.2450, 0.0916, 0.3872 });
            Console.WriteLine("Robot p calculated tool={0}", pose4_P_t);
            if(!AlmostEqualsMatrix(pose4.R, pose4_R_t, 1 * 10 ^ 4))
            { Console.WriteLine("FwdKin failed"); return; }
            if (!AlmostEqualsVector(pose4.P, pose4_P_t, 1 * 10 ^ 4))
            { Console.WriteLine("FwdKin failed"); return; }
            Console.WriteLine("FwdKin succeeded");
        }

        public static void RunTests()
        {
            TestHat();
            TestRot();
            TestR2Rot();
            TestScrewMatrix();
            TestR2Q();
            TestQ2R();
            TestRot2Q();
            TestQ2Rot();
            TestQuatcomplement();
            TestQuatproduct();
            TestQuatjacobian();
            TestRpy2R();
            Test_Subproblems();
            TestFwdkin();
        }
    }
}
