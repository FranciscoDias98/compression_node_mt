/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef OCTREE_COMPRESSION_H
#define OCTREE_COMPRESSION_H


#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/compression/entropy_range_coder.h>

//#include "entropy_range_coder.h"

#include "color_coding.h"

#include "point_coding.h"

#include "compression_profiles.h"

#include <iterator>
#include <iostream>
#include <vector>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iomanip>
//xxxxx
#include <inttypes.h>
#include <chrono>
#include <time.h>
#include<unistd.h>
#include <ros/ros.h>
#include <thread>

//unsigned long tempos_test_2 = 0;
//int counter = 0;

using namespace pcl::octree;

namespace pcl
{
  namespace io
  {
    /** \brief @b Octree pointcloud compression class
     *  \note This class enables compression and decompression of point cloud data based on octree data structures.
     *  \note
     *  \note typename: PointT: type of point used in pointcloud
     *  \author Julius Kammerl (julius@kammerl.de)
     */
    template<typename PointT, typename LeafT = OctreeContainerPointIndices,
        typename BranchT = OctreeContainerEmpty,
        typename OctreeT = Octree2BufBase<LeafT, BranchT> >
    class OctreePointCloudCompression : public OctreePointCloud<PointT, LeafT,
        BranchT, OctreeT>
    {
      public:
        // public typedefs
        typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
        typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
        typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT> > ConstPtr;

        typedef typename OctreeT::LeafNode LeafNode;
        typedef typename OctreeT::BranchNode BranchNode;

        typedef OctreePointCloudCompression<PointT, LeafT, BranchT, Octree2BufBase<LeafT, BranchT> > RealTimeStreamCompression;
        typedef OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeBase<LeafT, BranchT> > SinglePointCloudCompressionLowMemory;


        /** \brief Constructor
          * \param compressionProfile_arg:  define compression profile
          * \param octreeResolution_arg:  octree resolution at lowest octree level
          * \param pointResolution_arg:  precision of point coordinates
          * \param doVoxelGridDownDownSampling_arg:  voxel grid filtering
          * \param iFrameRate_arg:  i-frame encoding rate
          * \param doColorEncoding_arg:  enable/disable color coding
          * \param colorBitResolution_arg:  color bit depth
          * \param showStatistics_arg:  output compression statistics
          */
        OctreePointCloudCompression (compression_Profiles_e compressionProfile_arg = MED_RES_ONLINE_COMPRESSION_WITH_COLOR,
                               bool showStatistics_arg = false,
                               const double pointResolution_arg = 0.001,
                               const double octreeResolution_arg = 0.01,
                               bool doVoxelGridDownDownSampling_arg = false,
                               const unsigned int iFrameRate_arg = 30,
                               bool doColorEncoding_arg = true,
                               const unsigned char colorBitResolution_arg = 6) :
          OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (octreeResolution_arg),
          output_ (PointCloudPtr ()),
          binary_tree_data_vector_ (),
          binary_color_tree_vector_ (),
          point_count_data_vector_ (),
          point_count_data_vector_iterator_ (),
          color_coder_ (),
          point_coder_ (),
          entropy_coder_ (),
          do_voxel_grid_enDecoding_ (doVoxelGridDownDownSampling_arg), i_frame_rate_ (iFrameRate_arg),
          i_frame_counter_ (0), frame_ID_ (0), point_count_ (0), i_frame_ (true),
          do_color_encoding_ (doColorEncoding_arg), cloud_with_color_ (false), data_with_color_ (false),
          point_color_offset_ (0), b_show_statistics_ (showStatistics_arg), 
          compressed_point_data_len_ (), compressed_color_data_len_ (), selected_profile_(compressionProfile_arg),
          point_resolution_(pointResolution_arg), octree_resolution_(octreeResolution_arg),
          color_bit_resolution_(colorBitResolution_arg),
          object_count_(0)
        {
          initialization();
          std::cout << "olaaaaaaa" << std::endl;
        }

        /** \brief Empty deconstructor. */
        virtual
        ~OctreePointCloudCompression ()
        {
        }

        /** \brief Initialize globals */
        void initialization () {
          if (selected_profile_ != MANUAL_CONFIGURATION)
          {
            // apply selected compression profile

            // retrieve profile settings
            printf("tou nos profiles/n");
            const configurationProfile_t selectedProfile = compressionProfiles_[selected_profile_];

            // apply profile settings
            i_frame_rate_ = selectedProfile.iFrameRate;
            do_voxel_grid_enDecoding_ = selectedProfile.doVoxelGridDownSampling;
            this->setResolution (selectedProfile.octreeResolution);
            point_coder_.setPrecision (static_cast<float> (selectedProfile.pointResolution));
            do_color_encoding_ = selectedProfile.doColorEncoding;
            color_coder_.setBitDepth (selectedProfile.colorBitResolution);

          }
          else 
          {
            // configure point & color coder
            point_coder_.setPrecision (static_cast<float> (point_resolution_));
            color_coder_.setBitDepth (color_bit_resolution_);
          }

          if (point_coder_.getPrecision () == this->getResolution ())
            //disable differential point colding
            do_voxel_grid_enDecoding_ = true;

        }

        /** \brief Add point at index from input pointcloud dataset to octree
         * \param[in] pointIdx_arg the index representing the point in the dataset given by \a setInputCloud to be added
         */
        virtual void
        addPointIdx (const int pointIdx_arg)
        {
          ++object_count_;
          OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointIdx(pointIdx_arg);
        }

        /** \brief Provide a pointer to the output data set.
          * \param cloud_arg: the boost shared pointer to a PointCloud message
          */
        inline void
        setOutputCloud (const PointCloudPtr &cloud_arg)
        {
          std::cout << "output cloud" << std::endl;
          if (output_ != cloud_arg)
          {
            output_ = cloud_arg;
          }
        }

        /** \brief Get a pointer to the output point cloud dataset.
          * \return pointer to pointcloud output class.
          */
        inline PointCloudPtr
        getOutputCloud () const
        {
          return (output_);
        }

        /** \brief Encode point cloud to output stream
          * \param cloud_arg:  point cloud to be compressed
          * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
          */
        void
        encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);

        /////////////////////////////////////////////////////////////////////////////////////////////////

        //----------------------------------------------------------------------------------------------
        //---------------------------------- PEDREIRADAAAAAAAA ------------------------------------------
        void
        encodePointCloud_2 (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg)

        {
            std::cout << "//////////////// encodePointCloud_2 TESTE //////////////////\n";
            unsigned char recent_tree_depth =
                    static_cast<unsigned char> (this->getTreeDepth ());
            
            static int counter;
            static long tempos_test_2;


            auto start_setInputCloud = chrono::high_resolution_clock::now();
            // initialize octree
            this->setInputCloud (cloud_arg);
            printf("------ Done setInputCloud ------ \n ");
            auto stop_setInputCloud = chrono::high_resolution_clock::now();
            auto duration_setInputCloud = chrono::duration_cast<chrono::milliseconds>(stop_setInputCloud - start_setInputCloud);
            PCL_INFO("setInputCloud Time:  %ld ms \n",duration_setInputCloud);




            auto start_addPointsFromInputCloud = chrono::high_resolution_clock::now();
            // add point to octree
            this->addPointsFromInputCloud ();
            printf("------ Done addPointsFromInputCloud ------ \n ");
            auto stop_addPointsFromInputCloud = chrono::high_resolution_clock::now();
            auto duration_addPointsFromInputCloud = chrono::duration_cast<chrono::milliseconds>(stop_addPointsFromInputCloud - start_addPointsFromInputCloud);
            PCL_INFO("addPointsFromInputCloud Time:  %ld ms \n",duration_addPointsFromInputCloud);
            


            // make sure cloud contains points
            if (this->leaf_count_>0) {
                
                
                // color field analysis
                cloud_with_color_ = false;
                /*
                std::vector<pcl::PCLPointField> fields;
                int rgba_index = -1;
                rgba_index = pcl::getFieldIndex (*this->input_, "rgb", fields);
                if (rgba_index == -1)
                {
                    rgba_index = pcl::getFieldIndex (*this->input_, "rgba", fields);
                }
                if (rgba_index >= 0)
                {
                    point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
                    cloud_with_color_ = true;
                }
                */
                // apply encoding configuration
                cloud_with_color_ &= do_color_encoding_;

                
                // if octree depth changed, we enforce I-frame encoding
                i_frame_ |= (recent_tree_depth != this->getTreeDepth ());// | !(iFrameCounter%10);
                
                // enable I-frame rate
                if (i_frame_counter_++==i_frame_rate_)
                {
                    i_frame_counter_ =0;
                    i_frame_ = true;
                }
                
                // increase frameID
                frame_ID_++;
                
                // do octree encoding
                if (!do_voxel_grid_enDecoding_)
                {
                    point_count_data_vector_.clear ();
                    point_count_data_vector_.reserve (cloud_arg->points.size ());
                }
                
                // initialize color encoding
                color_coder_.initializeEncoding ();
                color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
                color_coder_.setVoxelCount (static_cast<unsigned int> (this->leaf_count_));
                
                // initialize point encoding
                point_coder_.initializeEncoding ();
                point_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
                



                // serialize octree
                if (i_frame_){
                    auto start_serializeTree = chrono::high_resolution_clock::now();
                    // i-frame encoding - encode tree structure without referencing previous buffer
                    this->serializeTree (binary_tree_data_vector_, false);
                    printf("------ Done serializeTree i-frame encoding ------ \n ");
                    auto stop_serializeTree = chrono::high_resolution_clock::now();
                    auto duration_serializeTree = chrono::duration_cast<chrono::milliseconds>(stop_serializeTree - start_serializeTree);
                    PCL_INFO("serializeTree Time:  %ld ms \n",duration_serializeTree);
                    //this->serializeTree2();
                }
                else{
                    // p-frame encoding - XOR encoded tree structure
                    auto start_serializeTree = chrono::high_resolution_clock::now();
                    this->serializeTree (binary_tree_data_vector_, true); // change to serializeTree2 <-----
                    printf("------ Done serializeTree  p-frame encoding - XOR ------ \n ");
                    auto stop_serializeTree = chrono::high_resolution_clock::now();
                    auto duration_serializeTree = chrono::duration_cast<chrono::milliseconds>(stop_serializeTree - start_serializeTree);
                    PCL_INFO("serializeTree Time:  %ld ms \n",duration_serializeTree);
                }


                ///////
                double min_x, min_y, min_z, max_x, max_y, max_z;

                this->getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

                printf("------ Bounding Box ------ \n ");

                printf("min_x: %f\n",min_x);
                printf("min_y: %f\n",min_y);
                printf("min_z: %f\n",min_z);
                printf("max_x: %f\n",max_x);
                printf("max_y: %f\n",max_y);
                printf("max_z: %f\n",max_z);

                //////// END MULTI
                ///

                PCL_INFO("Point count:  %ld \n",this->point_count_);
                PCL_INFO("Leaf count:  %ld \n",this->getLeafCount());
                PCL_INFO("Obj count:  %ld \n",this->object_count_);
                ///////////////// PRINT TO A FILE OCCUPANCY CODE /////////////////////

                // ************** IN BOTTOM ****************

                //////////////////////////////////////////////////////////////////////
                // write frame header information to stream
                this->writeFrameHeader (compressed_tree_data_out_arg);

                printf("------ Write frame header information to stream ------ \n ");

                // -------  Time to encode occupancy code  ------- //

                auto start = chrono::high_resolution_clock::now();
                // apply entropy coding to the content of all data vectors and send data to output stream
                this->entropyEncoding (compressed_tree_data_out_arg);
                printf("------ Done Entropy Encoding ------ \n ");
                auto stop = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
                PCL_INFO("Range Encoder Time:  %ld ms \n",duration);

                tempos_test_2 = tempos_test_2 + duration.count();
                counter++;
                if(counter==100){
                    counter=0;
                    PCL_INFO(" ---------------- Range Encoder Time:  %ld ms ----------------------- \n",tempos_test_2/100);
                }



                // ------------------------------------------------------------------
                // prepare for next frame
                this->switchBuffers ();
                
                // reset object count
                object_count_ = 0;
                
                if (b_show_statistics_)
                {
                    float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
                    float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);
                    PCL_INFO ("*** POINTCLOUD ENCODING *** 2\n");
                    PCL_INFO ("Frame ID: %d\n", frame_ID_);
                    if (i_frame_)
                        PCL_INFO ("Encoding Frame: Intra frame\n");
                    else
                        PCL_INFO ("Encoding Frame: Prediction frame\n");
                    PCL_INFO ("Number of encoded points: -------------------------%ld\n", point_count_);
                    PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
                    PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
                    PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
                    PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
                    PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
                    PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_color_data_len_) / 1024.0f);
                    PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ + bytes_per_color);
                    PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
                    PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
                }
                
                i_frame_ = false;
            } else {
                if (b_show_statistics_)
                    PCL_INFO ("Info: Dropping empty point cloud\n");
                this->deleteTree();
                i_frame_counter_ = 0;
                i_frame_ = true;
            }
        }


        // ******************** Tentativa de Multithreading ***********************
        //*************************************************************************
        void encodePointCloud_multi_thread(const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg,int thread_number){
            std::cout << "//////////////// encodePointCloud_multi_thread TESTE //////////////////\n";



            unsigned char recent_tree_depth =
                    static_cast<unsigned char> (this->getTreeDepth ());

            static int counter;
            static long tempos_test_2;


            auto start_setInputCloud = chrono::high_resolution_clock::now();
            // initialize octree
            this->setInputCloud (cloud_arg);
            printf("------ Done setInputCloud ------ \n ");
            auto stop_setInputCloud = chrono::high_resolution_clock::now();
            auto duration_setInputCloud = chrono::duration_cast<chrono::milliseconds>(stop_setInputCloud - start_setInputCloud);
            PCL_INFO("setInputCloud Time:  %ld ms \n",duration_setInputCloud);




            auto start_addPointsFromInputCloud = chrono::high_resolution_clock::now();
            // add point to octree
            this->addPointsFromInputCloud ();
            printf("------ Done addPointsFromInputCloud ------ \n ");
            auto stop_addPointsFromInputCloud = chrono::high_resolution_clock::now();
            auto duration_addPointsFromInputCloud = chrono::duration_cast<chrono::milliseconds>(stop_addPointsFromInputCloud - start_addPointsFromInputCloud);
            PCL_INFO("addPointsFromInputCloud Time:  %ld ms \n",duration_addPointsFromInputCloud);



            // make sure cloud contains points
            if (this->leaf_count_>0) {


                // color field analysis
                cloud_with_color_ = false;
                /*
            std::vector<pcl::PCLPointField> fields;
            int rgba_index = -1;
            rgba_index = pcl::getFieldIndex (*this->input_, "rgb", fields);
            if (rgba_index == -1)
            {
                rgba_index = pcl::getFieldIndex (*this->input_, "rgba", fields);
            }
            if (rgba_index >= 0)
            {
                point_color_offset_ = static_cast<unsigned char> (fields[rgba_index].offset);
                cloud_with_color_ = true;
            }
            */
                // apply encoding configuration
                cloud_with_color_ &= do_color_encoding_;


                // if octree depth changed, we enforce I-frame encoding
                i_frame_ |= (recent_tree_depth != this->getTreeDepth ());// | !(iFrameCounter%10);

                // enable I-frame rate
                if (i_frame_counter_++==i_frame_rate_)
                {
                    i_frame_counter_ =0;
                    i_frame_ = true;
                }

                // increase frameID
                frame_ID_++;

                // do octree encoding
                if (!do_voxel_grid_enDecoding_)
                {
                    point_count_data_vector_.clear ();
                    point_count_data_vector_.reserve (cloud_arg->points.size ());
                }

                // initialize color encoding
                color_coder_.initializeEncoding ();
                color_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));
                color_coder_.setVoxelCount (static_cast<unsigned int> (this->leaf_count_));

                // initialize point encoding
                point_coder_.initializeEncoding ();
                point_coder_.setPointCount (static_cast<unsigned int> (cloud_arg->points.size ()));




                // serialize octree
                if (i_frame_){
                    auto start_serializeTree = chrono::high_resolution_clock::now();
                    // i-frame encoding - encode tree structure without referencing previous buffer
                    this->serializeTree2 (binary_tree_data_vector_, false);
                    printf("------ Done serializeTree i-frame encoding ------ \n ");
                    auto stop_serializeTree = chrono::high_resolution_clock::now();
                    auto duration_serializeTree = chrono::duration_cast<chrono::milliseconds>(stop_serializeTree - start_serializeTree);
                    PCL_INFO("serializeTree Time:  %ld ms \n",duration_serializeTree);
                    //this->serializeTree2();
                }
                else{
                    // p-frame encoding - XOR encoded tree structure
                    auto start_serializeTree = chrono::high_resolution_clock::now();
                    this->serializeTree2 (binary_tree_data_vector_, true); // change to serializeTree2 <-----
                    printf("------ Done serializeTree  p-frame encoding - XOR ------ \n ");
                    auto stop_serializeTree = chrono::high_resolution_clock::now();
                    auto duration_serializeTree = chrono::duration_cast<chrono::milliseconds>(stop_serializeTree - start_serializeTree);
                    PCL_INFO("serializeTree Time:  %ld ms \n",duration_serializeTree);
                }

                ///////////////// PRINT TO A FILE OCCUPANCY CODE /////////////////////

                // ************** IN BOTTOM ****************

                PCL_INFO("Point count:  %ld \n",this->point_count_);
                PCL_INFO("Point count:  %ld \n",this->getLeafCount());
                //////////////////////////////////////////////////////////////////////
                // write frame header information to stream
                this->writeFrameHeader (compressed_tree_data_out_arg);

                printf("------ Write frame header information to stream ------ \n ");

                // -------  Time to encode occupancy code  ------- //

                auto start = chrono::high_resolution_clock::now();
                // apply entropy coding to the content of all data vectors and send data to output stream
                this->entropyEncoding (compressed_tree_data_out_arg);
                printf("------ Done Entropy Encoding ------ \n ");
                auto stop = chrono::high_resolution_clock::now();
                auto duration = chrono::duration_cast<chrono::milliseconds>(stop - start);
                PCL_INFO("Range Encoder Time:  %ld ms \n",duration);

                tempos_test_2 = tempos_test_2 + duration.count();
                counter++;
                if(counter==100){
                    counter=0;

                    PCL_INFO(" ---------------- Range Encoder Time:  %ld ms ----------------------- \n",tempos_test_2/100);
                }

                // ------------------------------------------------------------------
                // prepare for next frame
                this->switchBuffers ();

                // reset object count
                object_count_ = 0;

                if (b_show_statistics_)
                {
                    float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
                    float bytes_per_color = static_cast<float> (compressed_color_data_len_) / static_cast<float> (point_count_);
                    PCL_INFO ("*** POINTCLOUD ENCODING *** 2\n");
                    PCL_INFO ("Frame ID: %d\n", frame_ID_);
                    if (i_frame_)
                        PCL_INFO ("Encoding Frame: Intra frame\n");
                    else
                        PCL_INFO ("Encoding Frame: Prediction frame\n");
                    PCL_INFO ("Number of encoded points: -------------------------%ld\n", point_count_);
                    PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
                    PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
                    PCL_INFO ("Color compression percentage: %f%%\n", bytes_per_color / (sizeof (int)) * 100.0f);
                    PCL_INFO ("Color bytes per point: %f bytes\n", bytes_per_color);
                    PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
                    PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_color_data_len_) / 1024.0f);
                    PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ + bytes_per_color);
                    PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_color) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
                    PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_color));
                }

                i_frame_ = false;
            } else {
                if (b_show_statistics_)
                    PCL_INFO ("Info: Dropping empty point cloud\n");
                this->deleteTree();
                i_frame_counter_ = 0;
                i_frame_ = true;
            }
    }



        //*************************************************************************
        //*************************************************************************



          //----------------------------------------------------------------------------------------------
          //----------------------------------------------------------------------------------------------

        /** \brief Decode point cloud from input stream
          * \param compressed_tree_data_in_arg: binary input stream containing compressed data
          * \param cloud_arg: reference to decoded point cloud
          */
        void
        decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);

      //protected:

        /** \brief Write frame information to output stream
          * \param compressed_tree_data_out_arg: binary output stream
          */
        void
        writeFrameHeader (std::ostream& compressed_tree_data_out_arg);

        /** \brief Read frame information to output stream
          * \param compressed_tree_data_in_arg: binary input stream
          */
        void
        readFrameHeader (std::istream& compressed_tree_data_in_arg);

        /** \brief Synchronize to frame header
          * \param compressed_tree_data_in_arg: binary input stream
          */
        void
        syncToHeader (std::istream& compressed_tree_data_in_arg);

        /** \brief Apply entropy encoding to encoded information and output to binary stream
          * \param compressed_tree_data_out_arg: binary output stream
          */
        void
        entropyEncoding (std::ostream& compressed_tree_data_out_arg);

        /** \brief Entropy decoding of input binary stream and output to information vectors
          * \param compressed_tree_data_in_arg: binary input stream
          */
        void
        entropyDecoding (std::istream& compressed_tree_data_in_arg);

        /** \brief Encode leaf node information during serialization
          * \param leaf_arg: reference to new leaf node
          * \param key_arg: octree key of new leaf node
         */
        virtual void
        serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);

        /** \brief Decode leaf nodes information during deserialization
         * \param key_arg octree key of new leaf node
         */
        // param leaf_arg reference to new leaf node
        virtual void
        deserializeTreeCallback (LeafT&, const OctreeKey& key_arg);


        /** \brief Pointer to output point cloud dataset. */
        PointCloudPtr output_;

        /** \brief Vector for storing binary tree structure */
        std::vector<char> binary_tree_data_vector_;

        /** \brief Interator on binary tree structure vector */
        std::vector<char> binary_color_tree_vector_;

        /** \brief Vector for storing points per voxel information  */
        std::vector<unsigned int> point_count_data_vector_;

        /** \brief Interator on points per voxel vector */
        std::vector<unsigned int>::const_iterator point_count_data_vector_iterator_;

        /** \brief Color coding instance */
        ColorCoding<PointT> color_coder_;

        /** \brief Point coding instance */
        PointCoding<PointT> point_coder_;

        /** \brief Static range coder instance */
        StaticRangeCoder entropy_coder_;

        bool do_voxel_grid_enDecoding_;
        uint32_t i_frame_rate_;
        uint32_t i_frame_counter_;
        uint32_t frame_ID_;
        uint64_t point_count_;
        bool i_frame_;

        bool do_color_encoding_;
        bool cloud_with_color_;
        bool data_with_color_;
        unsigned char point_color_offset_;

        //bool activating statistics
        bool b_show_statistics_;
        uint64_t compressed_point_data_len_;
        uint64_t compressed_color_data_len_;

        // frame header identifier
        static const char* frame_header_identifier_;

        const compression_Profiles_e selected_profile_;
        const double point_resolution_;
        const double octree_resolution_;
        const unsigned char color_bit_resolution_;

        std::size_t object_count_;


        // My test Multithreading
        int number_threads;

        //vector<boost::thread *> thread_list;
        //boost::mutex mutex_cluster;
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster1,cluster2;
      };

    // define frame identifier
    template<typename PointT, typename LeafT, typename BranchT, typename OctreeT>
      const char* OctreePointCloudCompression<PointT, LeafT, BranchT, OctreeT>::frame_header_identifier_ = "<PCL-OCT-COMPRESSED>";
  }

}


#endif


//                FILE *file3,*file2,*file_1k;
//                //file3 = fopen("/home/francisco98/file/occupancy_code_HEX.txt","w"); // ---> imprime hex values
//                //file2 = fopen("/home/francisco98/file/occupancy_code_hex_line.txt","w"); // --->  imprime hex values \n
//                file_1k = fopen("/home/francisco98/file/occupancy_code_hex_1kbyte.txt","w");

//                ofstream my_file,my_file2,my_file3,my_file4;

//                //my_file.open("/home/francisco98/file/occupancy_code_bin.txt");
//                //my_file2.open("/home/francisco98/file/occupancy_code_bin_line.txt");
//                //my_file3.open("/home/francisco98/file/occupancy_code_hex.txt",ios::out | ios::binary);
//                //my_file4.open("/home/francisco98/file/occupancy_code_hex_line.txt",ios::out | ios::binary);
//                for(int i=0;i<1000;i++){
//                   //my_file << std::bitset<8>(binary_tree_data_vector_[i]) << std::endl;
//                   //my_file2 << std::bitset<8>(binary_tree_data_vector_[i]);
//                   //my_file3 << std::hex << binary_tree_data_vector_[i] << std::endl;
//                   //my_file4 << std::hex << binary_tree_data_vector_[i];
//                   if(binary_tree_data_vector_[i] < 0 )
//                       fprintf(file_1k,"%x\n",binary_tree_data_vector_[i] - 0xffffff00);
//                   else
//                       fprintf(file_1k,"%x\n",binary_tree_data_vector_[i]);


//                   //if(binary_tree_data_vector_[i] < 0 )
//                       //fprintf(file2,"%x",binary_tree_data_vector_[i] - 0xffffff00);
//                   //else
//                       //fprintf(file2,"%x",binary_tree_data_vector_[i]);
//                }

//                //my_file.close();
//                //my_file2.close();
//                //my_file3.close();
//                //my_file4.close();
//                //fclose(file3);
//                //fclose(file2);
//                fclose(file_1k);
//                ///// teste binario //////

//                std::cout << std::bitset<8>(binary_tree_data_vector_[0])  << std::endl;
//                std::cout << std::make_unsigned_t<int>(binary_tree_data_vector_[0]) << std::endl
