<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.8">
  <compound kind="file">
    <name>imu_identification_data_provider.h</name>
    <path>include/adi_imu/</path>
    <filename>imu__identification__data__provider_8h.html</filename>
    <class kind="class">ImuIdentificationDataProvider</class>
  </compound>
  <compound kind="file">
    <name>iio_wrapper.cpp</name>
    <path>src/</path>
    <filename>iio__wrapper_8cpp.html</filename>
    <member kind="define">
      <type>#define</type>
      <name>MAX_NO_OF_SAMPLES</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>a2caa0716e56afb87d491d87fb4460b62</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>buff_write_idx</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>a51e5bf2f116b1e09c8b9ed29bb4e326b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>buff_read_idx</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>a1532e6c5918d0afd3d2b45eaff3f6f68</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>buff_data</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>a9c684a2e63c59045ea4d135add263bb9</anchor>
      <arglist>[NO_OF_CHANS+1][MAX_NO_OF_SAMPLES]</arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>samp_freq</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>a4ce011ebc45fa44e68df76ab78144541</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>no_of_samp</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>ad00387e7a9a10b544928c5afc4ded5b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>uint32_t</type>
      <name>current_data_selection</name>
      <anchorfile>iio__wrapper_8cpp.html</anchorfile>
      <anchor>a60e90dbaa463bf8637eb6f63512b97d1</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>imu_control_parameters.cpp</name>
    <path>src/</path>
    <filename>imu__control__parameters_8cpp.html</filename>
  </compound>
  <compound kind="class">
    <name>AccelGyroTempDataProvider</name>
    <filename>classAccelGyroTempDataProvider.html</filename>
    <base>AccelGyroTempDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempDataProvider</name>
      <anchorfile>classAccelGyroTempDataProvider.html</anchorfile>
      <anchor>ad3afbaaf055db6e75d94205295f8290f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~AccelGyroTempDataProvider</name>
      <anchorfile>classAccelGyroTempDataProvider.html</anchorfile>
      <anchor>a46a788cd8ee9b14c04eb564003ddeb31</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classAccelGyroTempDataProvider.html</anchorfile>
      <anchor>af67881b4ea267e67689a2e5af672d947</anchor>
      <arglist>(adi_imu::msg::AccelGyroTempData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classAccelGyroTempDataProvider.html</anchorfile>
      <anchor>acb8ae19c02d11f1aec5c69c11638cc91</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AccelGyroTempDataProviderInterface</name>
    <filename>classAccelGyroTempDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempDataProviderInterface</name>
      <anchorfile>classAccelGyroTempDataProviderInterface.html</anchorfile>
      <anchor>a8c99006840c27be6ebc2c83f4296095b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AccelGyroTempDataProviderInterface</name>
      <anchorfile>classAccelGyroTempDataProviderInterface.html</anchorfile>
      <anchor>afb2081833893e32a460d658298989691</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classAccelGyroTempDataProviderInterface.html</anchorfile>
      <anchor>a0b20795091c2739841a9b08b647eb45b</anchor>
      <arglist>(adi_imu::msg::AccelGyroTempData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AccelGyroTempRosPublisher</name>
    <filename>classAccelGyroTempRosPublisher.html</filename>
    <base>AccelGyroTempRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempRosPublisher</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>af8fb4636878123e31f7b740c1f6a02f8</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~AccelGyroTempRosPublisher</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a074d147f93dc242ce7b254064f9b34ea</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>ade22dbc3766732e4bfa2281454ec0029</anchor>
      <arglist>(AccelGyroTempDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a4cfabbaac2eb9326afb28d95649c5154</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>AccelGyroTempDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a435e229211446240df0fa4cbecaba9b8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::AccelGyroTempData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a3ec3d01c3a5ba585e3e4284ab4ea3838</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::AccelGyroTempData</type>
      <name>m_message</name>
      <anchorfile>classAccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a99abaebe7b2e85d944980bf960777f02</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AccelGyroTempRosPublisherInterface</name>
    <filename>classAccelGyroTempRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempRosPublisherInterface</name>
      <anchorfile>classAccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a35273384fcd445e27fdfa7e4a72057b1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AccelGyroTempRosPublisherInterface</name>
      <anchorfile>classAccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a205416a2d172c6caf4a5f3d5a8417784</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classAccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>ae70930451d10d71f9417cfbeffc44592</anchor>
      <arglist>(AccelGyroTempDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classAccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>ac1f025c7a68632e7c702964d3fc946dd</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classAccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a3d5c5d96b9b34724e83f8a82d1755f8b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>AccelGyroTempSubscriberTest</name>
    <filename>classAccelGyroTempSubscriberTest.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>SetUpTestCase</name>
      <anchorfile>classAccelGyroTempSubscriberTest.html</anchorfile>
      <anchor>a47864f9624e8ecac8302dcaee645d4a1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>TearDownTestCase</name>
      <anchorfile>classAccelGyroTempSubscriberTest.html</anchorfile>
      <anchor>a419c199f9c6bbc610094323fb785eaac</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>IIOWrapper</name>
    <filename>classIIOWrapper.html</filename>
    <member kind="function">
      <type></type>
      <name>IIOWrapper</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ae9b4506c5a6cb9d56e36d11c3d4fc56e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~IIOWrapper</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab4e9232cf3ba9539db8e4bfac2e0ebf0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>createContext</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa56bb91153c46ef51c3d3d53a68cc423</anchor>
      <arglist>(const char *context)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>updateBuffer</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ae179fe95e80ad6624c6782de04c854ed</anchor>
      <arglist>(uint32_t data_selection)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopBufferAcquisition</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a807ef883b1b5a6119ce4ab75d21985c5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffLinearAccelerationX</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a24d03c22a08baf0c5209d7f31c9d0533</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffLinearAccelerationY</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a809c113401e38fdf0fc660735a2192e1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffLinearAccelerationZ</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a41e760380fa5036a652201e2df50c889</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffAngularVelocityX</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab98be528a5805d948e0cbf8c81bdac26</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffAngularVelocityY</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a530c457c0cd4d81951b7d174072fd75b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffAngularVelocityZ</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a4d5286b33d41295f1cc6567f67b4d6d2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffTemperature</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a8636bc7c28c829538d05d5bb638c9074</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getBuffSampleTimestamp</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a4a4cea0c85854aef0117ce98a587b8b4</anchor>
      <arglist>(int32_t &amp;sec, uint32_t &amp;nanosec)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedLinearAccelerationX</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ad1c329ac2d3785a4c4ceee754d5b1c7c</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedLinearAccelerationY</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a798839e37670a3ee83381c3b8e7bee9d</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedLinearAccelerationZ</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a63262c6685e600d505fc751db4f52f4e</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedAngularVelocityX</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a188e946663bc7b97c34051307f4d5d27</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedAngularVelocityY</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a1f3f03307a91115e17ee3f18323f061a</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedAngularVelocityZ</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>adc9f70fada12dbe6e110a8bcddf6c539</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaAngleX</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a68420aad14da02a7fd8316b6c7853c03</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaAngleY</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a58453b95cb9193b5b2813fd3c1f5e4d7</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaAngleZ</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a29160e4f1947315e1862bf1c1a0630bc</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaVelocityX</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a6ef7ac4cd9d2b1b7e0cd9a7a44e6dd35</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaVelocityY</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ad566dd23cded258dea92a16a88946902</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaVelocityZ</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a31a1dc7c03fc782b2eff9ae0f925e789</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedTemperature</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a23f8fbda995629b864dc72bcf3ba0984</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_x_calibbias</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a2cc98e4d4a651e82c7d5766f824acebd</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibbias_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a09fa5c163037f078a83e813ef13f73e9</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_y_calibbias</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aafec41ed9294cf0ef6ac0afaf350188e</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibbias_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a3730f3c7dcea63f4e2104a02c1e695ae</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_z_calibbias</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a84a5ecfbf0942fa175703f1a0aee8bcd</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibbias_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a640785a86e54405eecad55a93321bc26</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_x_calibbias</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a45cbee4a2eaf700b7396a4df956ed7e8</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibbias_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a371075c2f73786fc409dbfe11bb9b68f</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_y_calibbias</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab069f082479643718588962ed76e48e9</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibbias_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a6b81ddc7f9f7c67b15e4f8be594c0926</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_z_calibbias</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a777c78d431b5fc16088018b52ffa8d3a</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibbias_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a4265bcbf41f4c0582e08f30f5ab63eca</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>filter_low_pass_3db_frequency</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aee4ed59f8c41d2c1e1ad95d5179e0e61</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_filter_low_pass_3db_frequency</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab960b56f3b8fb3bab2f4d3e492616bca</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sampling_frequency</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ac8584252d4635e9a3c00f464cc176b17</anchor>
      <arglist>(double *result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_sampling_frequency</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a5cf1f0bfc5f5f0cf43c9707f399c2001</anchor>
      <arglist>(double val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_sensor_initialization_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ac818ead3ccd5f2a7c8ab59f5464cff9a</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_data_path_overrun</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aacdab905284a0a6ecabb1b9bcad81be9</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_automatic_reset</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa81282a7f07974ef69c65711200f5d7a</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_flash_memory_update_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9950dc38bc1e56a7fc42d36b50a3e718</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_spi_communication_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a1d23815c0de4a1ad4366602a4f4bdd96</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_crc_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a11b581d355f708591f0e425507d93164</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_standby_mode</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a88df8843b74de7c177dada4d61708fc4</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_sensor_self_test_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a6c7b7b8d0039a24ed6be2ce7b15fc814</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_flash_memory_test_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a39396f6d69a04202f9e859922f95fc34</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_clock_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a09e0a223d200b7d2ed1731f06635b370</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_gyroscope1_self_test_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aea94b1e579314660c7dd03d82ee4c29f</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_gyroscope2_self_test_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa85c0d7dbb01ff4201678d687b8adf93</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_acceleration_self_test_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a6f0b68bf91bb6bd000a570b349a4e9f4</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_x_axis_gyroscope_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a45143fbc3eba8260f9cda6870c704d19</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_y_axis_gyroscope_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab027802372011c7f7d9f59061017173c</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_z_axis_gyroscope_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>adc4a32825bc5982131f53e227805946b</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_x_axis_accelerometer_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>afd7976cbd87ab425eb6fdf1192e39e1b</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_y_axis_accelerometer_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a5e040011fba86b221a028f96440ac59e</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_z_axis_accelerometer_failure</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>adf039156a53b8bb0afebe4c48507c097</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_aduc_mcu_fault</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa691849d0baa2fad55deb7975a070734</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_flash_memory_write_count_exceeded_error</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab3a551d7da672e3c5d3cce79aac4a37c</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>gyroscope_measurement_range</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa3dfbe95f0013016b8b5056b73d1f126</anchor>
      <arglist>(std::string &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>internal_sensor_bandwidth</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a25e63a463289dc1c7761e9ef14982cbf</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_internal_sensor_bandwidth</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a630b1d5232285b51560a90b805339961</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>point_of_percussion_alignment</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a67eaded963621b47f1f228a0b250a00e</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_point_of_percussion_alignment</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a293d23f39cf1a07b3e8056d239728dc3</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>linear_acceleration_compensation</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a2352785dfb0a0324613bd6f99a27831b</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_linear_acceleration_compensation</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a09fe16394d6d8b7a7ab7dbefa64ba084</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>bias_correction_time_base_control</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a4b0d75039f36c1063c12c2335c230ffd</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_bias_correction_time_base_control</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a5818afde806f78c0868884d6b1f23bbf</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>x_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ab0d834e0075c408c546991934ac4ac7a</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_x_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>acedff444962fb642b5d2417aaf5dd63a</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>y_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a73fd94d29e90e9b46a5c47453e5a76a7</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_y_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a4a20c51a3a2079f1c7e887605f6f78dc</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>z_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a00aa6acb648e5c90038132c537649773</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_z_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a70f3f6d8b2cdefb6a5a6cff54915c6e1</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>x_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a62287c7a5dfdd731494ed7ba1780663d</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_x_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a43ae9191af8bc6da154d7b301848ac41</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>y_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9f393e7caf03730f7ddcd003c2556dfe</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_y_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ace85be9c516ad0dce1d76700c440d0e7</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>z_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a56ca0c938bef97c190ee7ae56736fcda</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_z_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ad501d5b8096992254282b710ae3593bf</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>bias_correction_update</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a2da8f4f3cd36e8e2cff6befcb53ecbbe</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>factory_calibration_restore</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a2cb403633337d848e3e334460098a954</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sensor_self_test</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9063348d525f98961fca0e5a8d1b2942</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>flash_memory_update</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a39973a796e6e9f809b8284f011ccaa36</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>flash_memory_test</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>af97abb3a23be70f995eb1becb1863dfb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>software_reset</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a06c8b2dd5e06296e6071515d42678338</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>firmware_revision</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a7ece889e328ab46d471031934eb93b33</anchor>
      <arglist>(std::string &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>firmware_date</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a392b3968397a63a0bf235bb7515f5879</anchor>
      <arglist>(std::string &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>product_id</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a1f0c7c86eba809ac5e0505b12e7b8143</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>serial_number</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9d05c33bbd4eef378b584d656a56cfc7</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>flash_counter</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>af2bd81f3dca62979b41cf5018f72a173</anchor>
      <arglist>(uint32_t &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_accel</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a45fe53b29bb3c69fd167caa42418b56e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_anglvel</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9155426d173e6adb8c6436598cab9d6b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_deltavelocity</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a0986611e409cfa128ec0f5b5429c8b72</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_deltaangl</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a133f8dcccf4727d7a97ba1f708c7c249</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_temp</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a022888d663a4dbead44e37a84c45411b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>setDeltaAngleScales</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aec9b699fd6110a7795a32e84343a1c1d</anchor>
      <arglist>(enum adis_device_id dev_id)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>setDeltaVelocityScales</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a7287fe1ab85c9aebb746dff3baf7300d</anchor>
      <arglist>(enum adis_device_id dev_id)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>updateField</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a3b99dc4a11bfc1044779befc29c9385a</anchor>
      <arglist>(uint32_t reg, uint32_t val, uint32_t mask)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaAngleXFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a47cb7f7d914c9a12c51cfaa152645b07</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaAngleYFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>adde2349c207c0d885197bb11165e6546</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaAngleZFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a2f8806cda1ba466fab265a45eb66815c</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaVelocityXFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a0fe6f4ff144f0c80f1452987e383318a</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaVelocityYFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ae6a0f742c9dd11684e9a49fb975773e0</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaVelocityZFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a153ba949b8daa8514864b59538fcb64a</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaAngleXFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a78ff447b7bccef0dd8abc5239ddcda27</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaAngleYFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa52383a8d31690b73f74539cc617e7d7</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaAngleZFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>adb513e9bdd99b7fcf9accec73faba53e</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaVelocityXFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>afae369c43ec0dfdb08f41e173b968f58</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaVelocityYFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a93c0f0e87624cc045bf571ed8d85911b</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaVelocityZFromDebug</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9adcb8752a2895affbab03ea3d1e2ef0</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_context *</type>
      <name>m_iio_context</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aef4fe3a3f93085da695bf65639f09b87</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_device *</type>
      <name>m_dev</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa54f78cb0c53f73b6562e303367b20be</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_device *</type>
      <name>m_dev_trigger</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a4c656678d45e8e978480712451b69649</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_buffer *</type>
      <name>m_dev_buffer</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ac6c3cec6da827146507437ee27296a96</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_accel_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ac1fb844f82d64a1b6e2e6cc6c089bd71</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_accel_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ad1e0779435221c690f93d6d05a76f176</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_accel_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa5f24dc01a83090154c57359a4668cd9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_anglvel_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9b7c68d2cd7161e853b261c3e1363ef3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_anglvel_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a1b138c4cc8238e52a68f830363a04b8d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_anglvel_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a1b2384d209272000f673f25e7134d06d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltaangl_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ae585f76ba5a1e003c9416bc87d5d7897</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltaangl_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a60cc79165edc458ad3af03ad976feb2d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltaangl_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a7f9f033ffb31d35240723713a9769a6b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltavelocity_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a13c3a4d31efdd2d329b3acc14b697aa4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltavelocity_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a7fba21deedbdc14adf0f4a46cb59fdff</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltavelocity_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a38e16f0f6f97028d0eee31b7e98d40aa</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_temp</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a49a1068fef0e2c1c48096f6b88d103b7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_timestamp</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a335fdbf54f6ea1f463b4b7e9e74e0cc8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_accel_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ac4a267304bf82229d032adf4c167c8be</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_accel_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a39297415934efc288f6ced765861d19d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_accel_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a62c70c9017644054942d0a27c5582291</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_anglvel_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a60201aaf24995e68520d846c5446cd23</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_anglvel_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aabb94da5ca0c007e066898cd7767f5a2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_anglvel_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>acdb2f23b02ac0650e12b76a02502e725</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltaangl_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a9d05021dd126ae2446d4bfc8e122d926</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltaangl_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a0a614ad2a77e11186da5c6d22743c1cc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltaangl_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>aa60f12b53d29ad7a17750da6cfe56a9b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltavelocity_x</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a6b769d1cfe0c7e2f09d7af4cdbea5c09</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltavelocity_y</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a5f29b5ac8ea9c50f38059626899f45ab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltavelocity_z</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>a822cfeb7635264d647df73dabd2ec4d6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_temp</name>
      <anchorfile>classIIOWrapper.html</anchorfile>
      <anchor>ac33c30566920a17f9f0b20321c7bb4ae</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuControlParameters</name>
    <filename>classImuControlParameters.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuControlParameters</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ac48f67e7a3b66410fb3c4e9ea0b5dc0a</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuControlParameters</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a9eab68cf68a1fba0e2d0ad312e814d37</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handleControlParams</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a3fdcf6864bdd68a06f712bef548acd24</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>UpdateUint32Params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a08337927ac5d0390c065a8f8b64e11ce</anchor>
      <arglist>)(uint32_t)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, UpdateUint32Params &gt;</type>
      <name>UpdateUint32ParamsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>add2e53a89e0f075df4a090fb28ea59d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>GetUint32Params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a82802c01a3698c99b5e0bbd5117f7dd1</anchor>
      <arglist>)(uint32_t &amp;)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, GetUint32Params &gt;</type>
      <name>GetUint32ParamsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>aca50889946b53f310ddd2cd1143c951b</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>UpdateInt32Params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a82366b8096ef75c05b44ece86798ecce</anchor>
      <arglist>)(int32_t)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, UpdateInt32Params &gt;</type>
      <name>UpdateInt32ParamsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a96d9f5b54857f9de53ab8aa9e28e1433</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>GetInt32Params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a87949036708585beab3c05aeff113f32</anchor>
      <arglist>)(int32_t &amp;)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, GetInt32Params &gt;</type>
      <name>GetInt32ParamsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a54231d8005b5513dc51d079e11b88cb7</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>UpdateDoubleParams</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ad9ec37d0885918285b76b0a01cc17ee8</anchor>
      <arglist>)(double)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, UpdateDoubleParams &gt;</type>
      <name>UpdateDoubleParamsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a217a4e02692f65433945b7a285b6415e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>GetDoubleParams</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a261cfd80e62c8a3362414fddd9f2da93</anchor>
      <arglist>)(double *)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, GetDoubleParams &gt;</type>
      <name>GetDoubleParamsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a619c7db2b3afced6b9fa7f6f770b9e17</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>ExecuteCommands</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>affa8e2f00d84a5f26a69f16eaf9f9c39</anchor>
      <arglist>)()</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, ExecuteCommands &gt;</type>
      <name>ExecuteCommandsMapType</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>af395e099df244af44911518e92f4a02b</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>declareAdisAttributes</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a7ddb2c0494555953182e42bd0078e8f2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOUpdateFunctionsInt32</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>acad11bd8e1ff8ef2d5bf512877ffa32f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOGetFunctionsInt32</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ad256c1c4af60a7fe2b6b6adbf1108e30</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOUpdateFunctionsUint32</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a0839165af2a0cc50e07ad6b55806953f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOGetFunctionsUint32</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>aff2a31343637952540bfe76a04728e51</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOUpdateFunctionsDouble</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ae6be8328df3865c9b057f7b6871612cd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOGetFunctionsDouble</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>acfe86a041e90121d3199ea7a0daaf8ac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOCommandFunctions</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a2895734d37c7278c6df4a51cbed6aa82</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>declareParameterDescription</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a3c2c658b2e5fb2e2fe659501e7203054</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>declareParameters</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ab8aba305fccae66c9ad5f4a0f9e0aece</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>updateParamsFromHardware</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>aa733955c32fc2253f4b14d2db1b454a2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleDoubleParamChange</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a747ae2b209292e2af2de26a8cc448c61</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleInt32ParamChange</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a3cd01af35e9eae635764fac2ed0bb3e2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleUint32ParamChange</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a117d2fb3cdb9cb7ae668295a7615df94</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleCommands</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ab456afd17772314a2bdcdb611aa6fc12</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a550af7cb014cc2b4dec9e42342303f63</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a1116f3c4083e6dc9dbea7e8104e6cde9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::string</type>
      <name>m_command_to_execute</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>abfb5fd3c8fad0ce52ac726cf135c5ebb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>UpdateUint32ParamsMapType</type>
      <name>m_func_map_update_uint32_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>af520e1d09d7b28678d5d3103cd1cb0d4</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>GetUint32ParamsMapType</type>
      <name>m_func_map_get_uint32_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>afd5ac3df4ab613f0b3f77775b8755c4f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>UpdateInt32ParamsMapType</type>
      <name>m_func_map_update_int32_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a27be5741591bf0fc60fe7167257ee112</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>GetInt32ParamsMapType</type>
      <name>m_func_map_get_int32_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ae8b7fda4cd0053f95fcdb17cd0d845e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>UpdateDoubleParamsMapType</type>
      <name>m_func_map_update_double_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a56ea60d73a130a69a0dd61c1d8779e27</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>GetDoubleParamsMapType</type>
      <name>m_func_map_get_double_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>af56801333a7647590e9318db88c65e85</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ExecuteCommandsMapType</type>
      <name>m_func_map_execute_commands</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ab5a51dfd26f3cdeeb7c2d1c1de771eb7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::list&lt; std::string &gt;</type>
      <name>m_attr_current_device</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ab1f1a2ace21a2123c9086c6e141713da</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, int32_t &gt;</type>
      <name>m_int32_current_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a30414b572a699d87f6d21e17a54a36ef</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, int64_t &gt;</type>
      <name>m_uint32_current_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a9bf1f32e4bbaaeba1a8d4d0bbc869ae2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, double &gt;</type>
      <name>m_double_current_params</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a7027df0c4d7cde53c39f46d3438d90e2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, std::string &gt;</type>
      <name>m_param_description</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>a793e0ffb6bfbea341b30d74ca854a585</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, rcl_interfaces::msg::IntegerRange &gt;</type>
      <name>m_param_constraints_integer</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>af0387f10c79cc939f530b9458b813a7a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, rcl_interfaces::msg::FloatingPointRange &gt;</type>
      <name>m_param_constraints_floating</name>
      <anchorfile>classImuControlParameters.html</anchorfile>
      <anchor>ab64303326f426a54dd55a201d8e8d411</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDataProvider</name>
    <filename>classImuDataProvider.html</filename>
    <base>ImuDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuDataProvider</name>
      <anchorfile>classImuDataProvider.html</anchorfile>
      <anchor>a7b58260bd62baf3000a1870ec0e07e67</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuDataProvider</name>
      <anchorfile>classImuDataProvider.html</anchorfile>
      <anchor>acfac62b4c0d1ba677cb6b31367545eec</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classImuDataProvider.html</anchorfile>
      <anchor>a9da652b7fbab948882f4bf6fcdf4c9f7</anchor>
      <arglist>(sensor_msgs::msg::Imu &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classImuDataProvider.html</anchorfile>
      <anchor>a1492459cc251d51ce04f5556864d4fbf</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDataProviderInterface</name>
    <filename>classImuDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuDataProviderInterface</name>
      <anchorfile>classImuDataProviderInterface.html</anchorfile>
      <anchor>afd0d8a28e9de26552c5cac982d85a39d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuDataProviderInterface</name>
      <anchorfile>classImuDataProviderInterface.html</anchorfile>
      <anchor>a09c5a4c79e9e97a40daf88b9ce7b57db</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classImuDataProviderInterface.html</anchorfile>
      <anchor>a308b5335b052855ce8917c2949ae1d8f</anchor>
      <arglist>(sensor_msgs::msg::Imu &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDiagDataProvider</name>
    <filename>classImuDiagDataProvider.html</filename>
    <base>ImuDiagDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuDiagDataProvider</name>
      <anchorfile>classImuDiagDataProvider.html</anchorfile>
      <anchor>a67f2a4fc39dccc3d5a3137bb77a4864b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuDiagDataProvider</name>
      <anchorfile>classImuDiagDataProvider.html</anchorfile>
      <anchor>a479edb5b4b2ab2159af4fde6d3c4ff27</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classImuDiagDataProvider.html</anchorfile>
      <anchor>a934cc7ba53b5e091d9e8f6fd55448ca2</anchor>
      <arglist>(adi_imu::msg::ImuDiagData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classImuDiagDataProvider.html</anchorfile>
      <anchor>ac2972e054da2eeda1558df42c7e4a8c9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDiagDataProviderInterface</name>
    <filename>classImuDiagDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuDiagDataProviderInterface</name>
      <anchorfile>classImuDiagDataProviderInterface.html</anchorfile>
      <anchor>a13231358624249ee9136d8e44fe35765</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuDiagDataProviderInterface</name>
      <anchorfile>classImuDiagDataProviderInterface.html</anchorfile>
      <anchor>acae6d777d7c6e038cec89826d9e9d1b2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classImuDiagDataProviderInterface.html</anchorfile>
      <anchor>a35be63cb4a4509a40d8af262420fb70f</anchor>
      <arglist>(adi_imu::msg::ImuDiagData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDiagRosPublisher</name>
    <filename>classImuDiagRosPublisher.html</filename>
    <base>ImuDiagRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuDiagRosPublisher</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>aef26c2843483d06048f716f4f2b8cf44</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuDiagRosPublisher</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>a6a4cf58c270529dcf9e02305d119e6ac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>ae5233d9e51106f0f5cf7a8734db34800</anchor>
      <arglist>(ImuDiagDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>aaf3056416be0e688ef7d3833c3bbfbfd</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuDiagDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>ae65f15d3729cdb044463a0ebdd2b4038</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::ImuDiagData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>a51c437ba1557e82b847b3bf86293e256</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::ImuDiagData</type>
      <name>m_message</name>
      <anchorfile>classImuDiagRosPublisher.html</anchorfile>
      <anchor>ae77e1ee412b353b9e66d3e490c518970</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDiagRosPublisherInterface</name>
    <filename>classImuDiagRosPublisherInterface.html</filename>
    <base>RosTask</base>
    <member kind="function">
      <type></type>
      <name>ImuDiagRosPublisherInterface</name>
      <anchorfile>classImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a5ed6cfb9a6c3962ac619cc6832b91b84</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuDiagRosPublisherInterface</name>
      <anchorfile>classImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>adc6b891a079402ab86761984c73bc508</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a1cf7bd52aaf259428911d0aa6eb2cd5e</anchor>
      <arglist>(ImuDiagDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a2410d6fb2cc843c224b1ed251ac92787</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuDiagSubscriberTest</name>
    <filename>classImuDiagSubscriberTest.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>SetUpTestCase</name>
      <anchorfile>classImuDiagSubscriberTest.html</anchorfile>
      <anchor>a1d24997802245f251c335872a82c4400</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>TearDownTestCase</name>
      <anchorfile>classImuDiagSubscriberTest.html</anchorfile>
      <anchor>ac47054a39336903feb6c016608add700</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuFullMeasuredDataProvider</name>
    <filename>classImuFullMeasuredDataProvider.html</filename>
    <base>ImuFullMeasuredDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataProvider</name>
      <anchorfile>classImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a8b4ccb7823f30176dc185c0b11991a8e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuFullMeasuredDataProvider</name>
      <anchorfile>classImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a44c5bb54a32d7f1514ba60687713ce1f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a5276a9d259d563868d3769d6ab8bd1c8</anchor>
      <arglist>(adi_imu::msg::ImuFullMeasuredData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a63ff7bc0aabb0df326fa9008790fe1b7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuFullMeasuredDataProviderInterface</name>
    <filename>classImuFullMeasuredDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataProviderInterface</name>
      <anchorfile>classImuFullMeasuredDataProviderInterface.html</anchorfile>
      <anchor>a9a2864395e9c525b0d3998271745c713</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuFullMeasuredDataProviderInterface</name>
      <anchorfile>classImuFullMeasuredDataProviderInterface.html</anchorfile>
      <anchor>aba9399db2895e334cc4dcbfa2157f187</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classImuFullMeasuredDataProviderInterface.html</anchorfile>
      <anchor>af94190681fdf6b4535410b905a8a3436</anchor>
      <arglist>(adi_imu::msg::ImuFullMeasuredData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuFullMeasuredDataRosPublisher</name>
    <filename>classImuFullMeasuredDataRosPublisher.html</filename>
    <base>ImuFullMeasuredDataRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>adb3eef4b042f4b4d45939d35486c59b2</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a8cf6829550691b33c0ca50272e8ade6d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a51c4795592fd41e712444552eeece31d</anchor>
      <arglist>(ImuFullMeasuredDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a18fa397d0a2774107bd74f29f5ecf08c</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuFullMeasuredDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>afc917159d4d339720979adbec90c2ef5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::ImuFullMeasuredData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>aa57d65687cb70be0fbcb367eec646eb3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::ImuFullMeasuredData</type>
      <name>m_message</name>
      <anchorfile>classImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a10286f173729d53b9fca60b9973d9417</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuFullMeasuredDataRosPublisherInterface</name>
    <filename>classImuFullMeasuredDataRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataRosPublisherInterface</name>
      <anchorfile>classImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>aa9ce565f5999ca1720fa6cefb0440e6d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuFullMeasuredDataRosPublisherInterface</name>
      <anchorfile>classImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>aad746b55a627f4acb1c50392e67db426</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>a799c51c1b501e469fa1e6f0dc77edc1a</anchor>
      <arglist>(ImuFullMeasuredDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>a4d4fd5aa71065f6ddc52fc8cc9ae06e2</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>a2643f9390c283e53bd23bf2978c1da34</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuFullMeasuredDataSubscriberTest</name>
    <filename>classImuFullMeasuredDataSubscriberTest.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>SetUpTestCase</name>
      <anchorfile>classImuFullMeasuredDataSubscriberTest.html</anchorfile>
      <anchor>ab55b195b93db47cf7b07156641fb06c7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>TearDownTestCase</name>
      <anchorfile>classImuFullMeasuredDataSubscriberTest.html</anchorfile>
      <anchor>a3c70b0c1048ab33ee5c1487bd8162828</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuIdentificationDataProvider</name>
    <filename>classImuIdentificationDataProvider.html</filename>
    <base>ImuIdentificationDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationDataProvider</name>
      <anchorfile>classImuIdentificationDataProvider.html</anchorfile>
      <anchor>a6b774b1a11a08fcdc6df1dfcb2b490d2</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuIdentificationDataProvider</name>
      <anchorfile>classImuIdentificationDataProvider.html</anchorfile>
      <anchor>ae4317cdb317f46f14a10723f767d7025</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classImuIdentificationDataProvider.html</anchorfile>
      <anchor>ae93f13d5654f5d83cb1b672b9984fbbb</anchor>
      <arglist>(adi_imu::msg::ImuIdentificationData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classImuIdentificationDataProvider.html</anchorfile>
      <anchor>ab2abdffd81df3c14185e5b9115fb26f6</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuIdentificationDataProviderInterface</name>
    <filename>classImuIdentificationDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationDataProviderInterface</name>
      <anchorfile>classImuIdentificationDataProviderInterface.html</anchorfile>
      <anchor>adc3d08505c9c45fd8ee2782963342812</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuIdentificationDataProviderInterface</name>
      <anchorfile>classImuIdentificationDataProviderInterface.html</anchorfile>
      <anchor>a3c20f4e190d9da755ee45499eba8dd54</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classImuIdentificationDataProviderInterface.html</anchorfile>
      <anchor>a1897dffdb20936fa8f25570eb767d9d5</anchor>
      <arglist>(adi_imu::msg::ImuIdentificationData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuIdentificationRosPublisher</name>
    <filename>classImuIdentificationRosPublisher.html</filename>
    <base>ImuIdentificationRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationRosPublisher</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>ae09df00ee5f8a7523e0da1682a10b596</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuIdentificationRosPublisher</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a1b39b89870753dd7594e216293ca8d8d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>ac4bc896dd849aa3e1c8363721a1c6238</anchor>
      <arglist>(ImuIdentificationDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a3c6ffc12c162c4ad6190e7b54b9a1bdc</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuIdentificationDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a46fd341acacf04dba8e5ad9258044fda</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::ImuIdentificationData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a4f092942baae2d6d3603968fea578435</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::ImuIdentificationData</type>
      <name>m_message</name>
      <anchorfile>classImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a10ec8befa2046eb720334b0bd4d395eb</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuIdentificationRosPublisherInterface</name>
    <filename>classImuIdentificationRosPublisherInterface.html</filename>
    <base>RosTask</base>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationRosPublisherInterface</name>
      <anchorfile>classImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>ae1d829038cd93abd04c4f4cabbe24de9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuIdentificationRosPublisherInterface</name>
      <anchorfile>classImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>a00337841abcc48e385e6c1f9f54defaa</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>af93cc79fb0cbed698b25662beefacceb</anchor>
      <arglist>(ImuIdentificationDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>ae4b1f479fbe79db85ae9df56ca41a675</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuIdentificationSubscriberTest</name>
    <filename>classImuIdentificationSubscriberTest.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>SetUpTestCase</name>
      <anchorfile>classImuIdentificationSubscriberTest.html</anchorfile>
      <anchor>a32056248dd3fcf21fe97f2b5b41dae8d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>TearDownTestCase</name>
      <anchorfile>classImuIdentificationSubscriberTest.html</anchorfile>
      <anchor>a3fcc6269d08ee482073a68eae77229e8</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuRosPublisher</name>
    <filename>classImuRosPublisher.html</filename>
    <base>ImuRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuRosPublisher</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>a4706d76ef778af74fc42f0e1987e39e5</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuRosPublisher</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>a0003f105376ae33e60ce10688d85a08b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>a78dc9501d090b81ff7de5c5f1f0d765e</anchor>
      <arglist>(ImuDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>aa6968af9b20c3a0616cbeafc0e5a8eda</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>a436a9b31a918449c9d30daf0664b2f60</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; sensor_msgs::msg::Imu &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>a21ebaa5902f0dd8f61567b1e56be4a71</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>sensor_msgs::msg::Imu</type>
      <name>m_message</name>
      <anchorfile>classImuRosPublisher.html</anchorfile>
      <anchor>ac569c410dbef56fb611d3a11a76a78b8</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuRosPublisherInterface</name>
    <filename>classImuRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuRosPublisherInterface</name>
      <anchorfile>classImuRosPublisherInterface.html</anchorfile>
      <anchor>a31b100cb8a9da5c0f385f1438c04a64b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuRosPublisherInterface</name>
      <anchorfile>classImuRosPublisherInterface.html</anchorfile>
      <anchor>a7867f11bfbb64c03895533c8d627cb3f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classImuRosPublisherInterface.html</anchorfile>
      <anchor>a89a2e7b14c3bf6e31fd2c69f2be492f8</anchor>
      <arglist>(ImuDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classImuRosPublisherInterface.html</anchorfile>
      <anchor>aeda9158c26d62348fcb83181103862ce</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classImuRosPublisherInterface.html</anchorfile>
      <anchor>a1db80017d3556a4a8efb1971b187d182</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>ImuSubscriberTest</name>
    <filename>classImuSubscriberTest.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>SetUpTestCase</name>
      <anchorfile>classImuSubscriberTest.html</anchorfile>
      <anchor>a5013c2c26c6e8af50ecf54af5be5274f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>TearDownTestCase</name>
      <anchorfile>classImuSubscriberTest.html</anchorfile>
      <anchor>a15eebc7f09bb8a928e69c24fd64a43cc</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>RosPublisherGroup</name>
    <filename>classRosPublisherGroup.html</filename>
    <base>RosPublisherGroupInterface</base>
    <member kind="function">
      <type></type>
      <name>RosPublisherGroup</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a77dd776062db65bedf195c431b852bf2</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~RosPublisherGroup</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>aa9a84b81b952bd75cf87bb5c0e73f40a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setAccelGyroTempRosPublisher</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a52fea1a71e98cb0625c889b31eebb6fc</anchor>
      <arglist>(AccelGyroTempRosPublisherInterface *accelGyroTempRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImuRosPublisher</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a7c1b3b2b98a90f8484da38fb24a559a7</anchor>
      <arglist>(ImuRosPublisherInterface *imuRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a124b01689fb74a813d7af292aa9dd065</anchor>
      <arglist>(ImuFullMeasuredDataRosPublisherInterface *imuFullMeasuredDataRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImuControlParameters</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a9b1f3c0218ffa6ff7d2fb6f6870b72fb</anchor>
      <arglist>(ImuControlParameters *imuControlParameters) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>afee0f211b77283a37fa197b39f4d1253</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>AccelGyroTempRosPublisherInterface *</type>
      <name>m_accelGyroTempRosPublisher</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a0ffa2985f2e02ca1139453ad7ed2b3df</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuRosPublisherInterface *</type>
      <name>m_imuRosPublisher</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>afc3e8969f04fc70a2f52a0e88f3a645a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuFullMeasuredDataRosPublisherInterface *</type>
      <name>m_imuFullMeasuredDataRosPublisher</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a00a053daf7455c5024f16b686d524676</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuControlParameters *</type>
      <name>m_imuControlParameters</name>
      <anchorfile>classRosPublisherGroup.html</anchorfile>
      <anchor>a3b53e5abac921148bf0c248a8cd07596</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>RosPublisherGroupInterface</name>
    <filename>classRosPublisherGroupInterface.html</filename>
    <base>RosTask</base>
    <member kind="function">
      <type></type>
      <name>RosPublisherGroupInterface</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>aaf8be6806a488d5f4ee676399e64774c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RosPublisherGroupInterface</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>a4c3de3a49b137a54288a118bee283afc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setAccelGyroTempRosPublisher</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>a30bd6c52a5e925f37b1613230e9f9b5a</anchor>
      <arglist>(AccelGyroTempRosPublisherInterface *accelGyroTempRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setImuRosPublisher</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>a5ff386378b8ab64f63162e8582c8880f</anchor>
      <arglist>(ImuRosPublisherInterface *imuRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>a0d0f06f5f5cde238b42904d080fe44d3</anchor>
      <arglist>(ImuFullMeasuredDataRosPublisherInterface *imuFullMeasuredDataRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setImuControlParameters</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>afec58a4c181b0737f9385a58b5c06178</anchor>
      <arglist>(ImuControlParameters *imuControlParameters)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classRosPublisherGroupInterface.html</anchorfile>
      <anchor>af8621aa2254eb8b90a39349559293d5b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>RosTask</name>
    <filename>classRosTask.html</filename>
    <member kind="function">
      <type></type>
      <name>RosTask</name>
      <anchorfile>classRosTask.html</anchorfile>
      <anchor>a519dd722e83fba675d0e988c5d5100d1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RosTask</name>
      <anchorfile>classRosTask.html</anchorfile>
      <anchor>a1043013f8bb20f9c3878d4cb8d99b2fb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>run</name>
      <anchorfile>classRosTask.html</anchorfile>
      <anchor>a331afbcd684711c35e1488fbaad73cf2</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>VelAngTempDataProvider</name>
    <filename>classVelAngTempDataProvider.html</filename>
    <base>VelAngTempDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>VelAngTempDataProvider</name>
      <anchorfile>classVelAngTempDataProvider.html</anchorfile>
      <anchor>ae94b88e60be8b88f38ce395c93b22d4c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~VelAngTempDataProvider</name>
      <anchorfile>classVelAngTempDataProvider.html</anchorfile>
      <anchor>aaa957dfc12e03ccdf7453b8e98b34573</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classVelAngTempDataProvider.html</anchorfile>
      <anchor>a3a41a957fd644e599139450d5abffb9f</anchor>
      <arglist>(adi_imu::msg::VelAngTempData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classVelAngTempDataProvider.html</anchorfile>
      <anchor>a1c8afa5aad73fd438814807e594df22d</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>VelAngTempDataProviderInterface</name>
    <filename>classVelAngTempDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>VelAngTempDataProviderInterface</name>
      <anchorfile>classVelAngTempDataProviderInterface.html</anchorfile>
      <anchor>a45fd34f04fab06fdca49c5625569ce35</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~VelAngTempDataProviderInterface</name>
      <anchorfile>classVelAngTempDataProviderInterface.html</anchorfile>
      <anchor>a07810df68e3caef91f8fb9dc4815f02e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classVelAngTempDataProviderInterface.html</anchorfile>
      <anchor>a209118f87b8ae0b7a2a2085f3c9c95e0</anchor>
      <arglist>(adi_imu::msg::VelAngTempData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>VelAngTempRosPublisher</name>
    <filename>classVelAngTempRosPublisher.html</filename>
    <base>VelAngTempRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>VelAngTempRosPublisher</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>ad222a6358968546c8a5eb6c4a8102ef3</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~VelAngTempRosPublisher</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>a38d6d1e83f097b9975054aa2bdd2147a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>a539b342de4cc1eeefb0591fe731950e1</anchor>
      <arglist>(VelAngTempDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>a3729886cf0214c86a6ee9296d58c0dc3</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>VelAngTempDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>ad029ab5a35bccd34fd0174b6aea7b824</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::VelAngTempData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>a88bfcbfc561ed2fe52111b6f708ef1af</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::VelAngTempData</type>
      <name>m_message</name>
      <anchorfile>classVelAngTempRosPublisher.html</anchorfile>
      <anchor>a716b7f2269ee985456060e621d2b5f6b</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>VelAngTempRosPublisherInterface</name>
    <filename>classVelAngTempRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>VelAngTempRosPublisherInterface</name>
      <anchorfile>classVelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>ac4840670a590e00d6a6f9108e81db019</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~VelAngTempRosPublisherInterface</name>
      <anchorfile>classVelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>a36feb8b97fbd8e2ea430d4609c4f62e1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classVelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>a5fb51d73442921eb29f3052b8db018a9</anchor>
      <arglist>(VelAngTempDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classVelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>a3bf1cfbe81422dbbe70bf42cafe78b69</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classVelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>a70429e77f0f6a5715b8bf399d0b14626</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>VelAngTempSubscriberTest</name>
    <filename>classVelAngTempSubscriberTest.html</filename>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>SetUpTestCase</name>
      <anchorfile>classVelAngTempSubscriberTest.html</anchorfile>
      <anchor>a6644c917bb2f0286416a9f4295028977</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" static="yes">
      <type>static void</type>
      <name>TearDownTestCase</name>
      <anchorfile>classVelAngTempSubscriberTest.html</anchorfile>
      <anchor>abfd9acc8721263a9c7106e1b2d69886f</anchor>
      <arglist>()</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>WorkerThread</name>
    <filename>classWorkerThread.html</filename>
    <member kind="function">
      <type></type>
      <name>WorkerThread</name>
      <anchorfile>classWorkerThread.html</anchorfile>
      <anchor>a942f957a04f6d5a7b7677836ad2f7650</anchor>
      <arglist>(RosTask *rosTask)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~WorkerThread</name>
      <anchorfile>classWorkerThread.html</anchorfile>
      <anchor>a5aa554e7d73f0f185a850ca0ef145d6d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>runTask</name>
      <anchorfile>classWorkerThread.html</anchorfile>
      <anchor>a15c7f87a0cf955ef034ec99f9ccb22ff</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>RosTask *</type>
      <name>m_rosTask</name>
      <anchorfile>classWorkerThread.html</anchorfile>
      <anchor>a51d45d8d09d7826fe6e9ac5fa31dceec</anchor>
      <arglist></arglist>
    </member>
  </compound>
</tagfile>
