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
#include <fm_rviz_polygon_tool/polygon_pannel.h>

namespace rviz_plugin_pannel
{

PolygonPanel::PolygonPanel( QWidget* parent )
  : rviz::Panel( parent )
{  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Output Topic polygon_pannel:" ));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );

  save_button_ = new QPushButton(tr("Save..."));
  save_button_->setToolTip(tr("Save mission to a file."));
  save_button_->setEnabled(true);

  new_poly_button_ = new QPushButton(tr("New polygon"));
  new_poly_button_->setToolTip(tr("Create a new polygon."));
  new_poly_button_->setEnabled(true);

  QHBoxLayout* save_file_layout = new QHBoxLayout;
  // save_file_layout->addWidget(save_button_);
  save_file_layout->addWidget(new_poly_button_);
  layout->addLayout(save_file_layout);

  setLayout(layout);

  connect(new_poly_button_, SIGNAL(clicked()), this, SLOT(createNewPoly()));
}

void PolygonPanel::createNewPoly(){
  std::cout<<"clicked on createNewPoly"<<std::endl;
  //tool_manager_
  //polygn_tl_ = new mav_coverage_planning::PolygonTool();
  //->onInitialize();
  //polygn_tl_->activate();
}

void PolygonPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void PolygonPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    // output_topic_editor_->setText( topic );
    // updateTopic();
  }
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class
#include <pluginlib/class_list_macros.h>
#include <rviz/panel.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_pannel::PolygonPanel, rviz::Panel )
