<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile doxygen_version="1.9.8">
  <compound kind="class">
    <name>adi_imu::AccelGyroTempDataProvider</name>
    <filename>classadi__imu_1_1AccelGyroTempDataProvider.html</filename>
    <base>adi_imu::AccelGyroTempDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempDataProvider</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProvider.html</anchorfile>
      <anchor>afa8bd0ede63f8b148e9d5d4c155f4374</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~AccelGyroTempDataProvider</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProvider.html</anchorfile>
      <anchor>a07122308ad8ff0192521da47c9e5c3e9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProvider.html</anchorfile>
      <anchor>ab0354f3fce8cf368b11df2aba7bcd12d</anchor>
      <arglist>(adi_imu::msg::AccelGyroTempData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProvider.html</anchorfile>
      <anchor>a5c88e913d91ad3c7439a5c96a2c37afb</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::AccelGyroTempDataProviderInterface</name>
    <filename>classadi__imu_1_1AccelGyroTempDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProviderInterface.html</anchorfile>
      <anchor>a350640588bfa7a2ca3fbc669be115ed1</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AccelGyroTempDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProviderInterface.html</anchorfile>
      <anchor>a25d2c1eb8ba0bcdc965d05ff2e5487cc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempDataProviderInterface.html</anchorfile>
      <anchor>a6c81453d162fcb32a7c9ab37364c195b</anchor>
      <arglist>(adi_imu::msg::AccelGyroTempData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::AccelGyroTempRosPublisher</name>
    <filename>classadi__imu_1_1AccelGyroTempRosPublisher.html</filename>
    <base>adi_imu::AccelGyroTempRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>aff9ccd12afc92de9a41463edebfc7594</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~AccelGyroTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>ae757fad91cbf739270084bfe7f0c68ae</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>ac2ae64fe792cb88a5921abe068f395af</anchor>
      <arglist>(AccelGyroTempDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>ab6cea9e16cf5842894db9f389302fd94</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>AccelGyroTempDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a53e26314b69d2ecb0a0a5b3bf403dcb1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::AccelGyroTempData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>ac7e253ea51a397a6ff2ae8a9f21ee676</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::AccelGyroTempData</type>
      <name>m_message</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisher.html</anchorfile>
      <anchor>a8c4c9f1919aa59aea9a2e216a1ba1e39</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::AccelGyroTempRosPublisherInterface</name>
    <filename>classadi__imu_1_1AccelGyroTempRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>AccelGyroTempRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a989dd5be98341d897804b260ad204302</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~AccelGyroTempRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>ac3d29950dd84a23b5e58cc2f6f0b3f1d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a3a9f758da0865eb864787c8adbf8d58a</anchor>
      <arglist>(AccelGyroTempDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a31ab9cae4c33db3e483b1bff951b6cf3</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1AccelGyroTempRosPublisherInterface.html</anchorfile>
      <anchor>a3980b78df28495efa342af1defe7eb0e</anchor>
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
    <name>adi_imu::Adis1646xRegisterMap</name>
    <filename>classadi__imu_1_1Adis1646xRegisterMap.html</filename>
    <base>adi_imu::ADISRegisterMap</base>
  </compound>
  <compound kind="class">
    <name>adi_imu::Adis1647xRegisterMap</name>
    <filename>classadi__imu_1_1Adis1647xRegisterMap.html</filename>
    <base>adi_imu::ADISRegisterMap</base>
  </compound>
  <compound kind="class">
    <name>adi_imu::Adis1650xRegisterMap</name>
    <filename>classadi__imu_1_1Adis1650xRegisterMap.html</filename>
    <base>adi_imu::ADISRegisterMap</base>
  </compound>
  <compound kind="class">
    <name>adi_imu::Adis1654xRegisterMap</name>
    <filename>classadi__imu_1_1Adis1654xRegisterMap.html</filename>
    <base>adi_imu::ADISRegisterMap</base>
  </compound>
  <compound kind="class">
    <name>adi_imu::Adis1655xRegisterMap</name>
    <filename>classadi__imu_1_1Adis1655xRegisterMap.html</filename>
    <base>adi_imu::ADISRegisterMap</base>
  </compound>
  <compound kind="class">
    <name>adi_imu::Adis1657xRegisterMap</name>
    <filename>classadi__imu_1_1Adis1657xRegisterMap.html</filename>
    <base>adi_imu::ADISRegisterMap</base>
  </compound>
  <compound kind="class">
    <name>adi_imu::ADISDeviceFactory</name>
    <filename>classadi__imu_1_1ADISDeviceFactory.html</filename>
  </compound>
  <compound kind="class">
    <name>adi_imu::ADISDeviceRegistry</name>
    <filename>classadi__imu_1_1ADISDeviceRegistry.html</filename>
  </compound>
  <compound kind="class">
    <name>adi_imu::ADISRegisterMap</name>
    <filename>classadi__imu_1_1ADISRegisterMap.html</filename>
  </compound>
  <compound kind="class">
    <name>adi_imu::IIOWrapper</name>
    <filename>classadi__imu_1_1IIOWrapper.html</filename>
    <member kind="function">
      <type></type>
      <name>IIOWrapper</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab5131a6020614c13c22814a1ccc6c662</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~IIOWrapper</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ad47271d924338efd29a2f92a64d314c0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setDeviceDescriptor</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a250ed73b89e7ddc5e6384235d8a1b05d</anchor>
      <arglist>(std::shared_ptr&lt; ADISRegisterMap &gt; device_descriptor)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>createContext</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ad1cd18a7c632c7f6f93be1d99b98222d</anchor>
      <arglist>(const char *context)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>updateBuffer</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af8d7480b3372bbe60162678179cebe99</anchor>
      <arglist>(uint32_t data_selection)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>stopBufferAcquisition</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7c4493da6ce8775c05a9ce0d46c8cd5d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffLinearAccelerationX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a942884b4879f2880372c036b402090eb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffLinearAccelerationY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aad735d96e2cb2b72edffba846cc3cbdd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffLinearAccelerationZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a9ac41732a91463fc653ba1f747857f46</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffAngularVelocityX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a922fa1b16df69f89f81ff481630eb15b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffAngularVelocityY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8e07ce559d0f884d5bd20f42cb5656eb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffAngularVelocityZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ada08f53c79ad7b276378e1aca209c466</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffDeltaVelocityX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0bb94b79b440601656ded7760ae885ef</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffDeltaVelocityY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>abb2002fa4c32a8b7ccacf4ef838ebb4e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffDeltaVelocityZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1d7cffbbeb5bed9b472bb79abcda5820</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffDeltaAngleX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aac81a38d455e1591de67ec8dab277a57</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffDeltaAngleY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ac7c21a78fba5c68932f7e06ff9f09601</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffDeltaAngleZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae5c229e8808f9c41ba18c6a71255e695</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>getBuffTemperature</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a754483b36d623d88af1a89b83d27cc04</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>getBuffSampleTimestamp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a03cacaeb38ee3df0b6dac3be369b3ef9</anchor>
      <arglist>(int32_t &amp;sec, uint32_t &amp;nanosec)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedLinearAccelerationX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ac8ae8162d8d0b4ea46871a8c06e959c7</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedLinearAccelerationY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a95bfba2f093abc41fd6b4850f9a7bc45</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedLinearAccelerationZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a127cc7abd87d2b0283ed4a047e3595b5</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedAngularVelocityX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af084ac4640bcd6284dc3e09fecf14042</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedAngularVelocityY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>adb3c808cc553e063d329c42ec4b7aa88</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedAngularVelocityZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a129ac37d3df127b8971952fa269ad467</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaAngleX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af01bd45df04139f1f0ec551748948ccf</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaAngleY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>abc619ae556e746200f7195ebd412817c</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaAngleZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a3ca9443d17623ca6ab66635b3a6ddc45</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaVelocityX</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa2940fe66d0e3439c63c80fa44a10ddb</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaVelocityY</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>afc627f787aca37937d4af7ab4ebdb910</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedDeltaVelocityZ</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a212b22599d909baf66f5d608efaac66a</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getConvertedTemperature</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>abb405d9d4dc0723cfdc563a7d402ba95</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_x_calibbias</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a6171bb87e271b31656a05418d087d1e0</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibbias_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8511cbbafba62c2af53a475816ec04b2</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_y_calibbias</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a38563c97835874f0f394ae3c17531981</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibbias_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>afb36d451e3c9d9a190afeba946ee66f6</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_z_calibbias</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ac1ef5a5e222021049fbaa1a63d93c670</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibbias_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a3fd7719cc0d8b9514987017470c64bbf</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_x_calibbias</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5e7520019943220e596822b6ffe25327</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibbias_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a07e9353509196f895f3fe912b797fe72</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_y_calibbias</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab53cc9a4c40009cbc2f218c673caedcf</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibbias_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>afd7add882617386b5a7332b12f393222</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_z_calibbias</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aaaba3aea3f37c55916e2ff043e96adf3</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibbias_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab73edade6388423f78d8b7f150ed0cb2</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>angvel_x_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>afd224821513b63e6d91cb6a41ebbb295</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_angvel_x_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a70471e4dcf0cc7fa9d3b83df54809b29</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>angvel_y_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7a78d44693dd9fe0a015ca63c0560ee6</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_angvel_y_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5685c918fdb51fb57e989b0f95226526</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>angvel_z_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a04c0b620a8764758adcc8d09aa08f4fe</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_angvel_z_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a2c7f40149961e45d5a3c20054a46da6b</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_x_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a93e82f911c588b1d2a67026baea810e7</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_x_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aec51d9faf7f8fe107cb7a73e746d080e</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_y_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ac7dbc0490cb70ed2f4c9e87b03dcec48</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_y_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a01de4f4945b704578a0996dfbbb7ab39</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_z_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a2a80a31272bd170eacc79e7733556dba</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_z_filter_low_pass_3db</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1c5554266196dcaf4f928e0e64cc3b85</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>filter_low_pass_3db_frequency</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a45411a358175371f8fac0c4cbb4be3e2</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_filter_low_pass_3db_frequency</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a00b0c9db656e7a03fb64d42724767208</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_x_calibscale</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a594cd8ddecea7b46ab3ba17239beba5f</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_y_calibscale</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa514e1b6c4a49884b3828abe0ce20ee0</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>accel_z_calibscale</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab6062360d1051b00ef1ad5deddbcfb70</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_x_calibscale</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab3eec0ad69076605ac84b6f8e9141eae</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_y_calibscale</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a2b158e5236250cf36ac66ae81a9933f0</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>anglvel_z_calibscale</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aba3d8bdca87b483ba8dc0946226f37c8</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibscale_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a97c1e131c865f15e625d327b35b460e1</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibscale_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a17eacbf64bd26ccf8b2d524c5abaa565</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_accel_calibscale_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af488a9fac2373e80407b737f5d7cca82</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibscale_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af09465ae110a9584820fcf20276145b2</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibscale_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a335ad3cfaac66041d2157640d878c7fc</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_anglvel_calibscale_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af97f7baefdc0c01d1f6e4ebfe08aa519</anchor>
      <arglist>(int32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sampling_frequency</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0c0a937542d69b7448bf7230d710f4f0</anchor>
      <arglist>(double *result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_sampling_frequency</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0d6c450a1ac2b8dbde4592bc311f29c3</anchor>
      <arglist>(double val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_sensor_initialization_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>abca18848f050647f213fb8c51745db8b</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_data_path_overrun</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a93b5eebe38ebadd4aba1431771097188</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_automatic_reset</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a59b878c5b0e55877f544830ab16c1fab</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_flash_memory_update_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ad1bea0105e626897a7738bbbdf22e3c6</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_spi_communication_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab2b7290458b29cd020269dd9ce53ad92</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_crc_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ada96236f0c0a09fbd35fa575a3d1048d</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_standby_mode</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a2fb5bdf71bb3b138a1c910560e619f8e</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_sensor_self_test_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>add78686902476e6bb0798e39fe5e5e05</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_flash_memory_test_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>abb5db9cef417d8914efad3f3b14dde39</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_clock_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a705334d453f4204e6efcd6cdcf624c4c</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_gyroscope1_self_test_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7bc754da843132a7ede91f0e517fc66f</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_gyroscope2_self_test_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a58a64401a07cadd636b73c427de18c7c</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_acceleration_self_test_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af4d7afbff5904f840d846d7bb195b533</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_x_axis_gyroscope_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae06a2253519b7da21e48fa02228e4870</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_y_axis_gyroscope_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae9eb1d8c8164ee01a4759eefac4c3e0b</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_z_axis_gyroscope_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aada937da8bc7c4d4cf8041977e415f9f</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_x_axis_accelerometer_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1af58e833e23588e05edef6f51371fc3</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_y_axis_accelerometer_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a3604f94044488e3b76e2a0105a956ecd</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_z_axis_accelerometer_failure</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>add0f460dc4f577c3894514d017b354a4</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_aduc_mcu_fault</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a09b63665e29322623937154c307e1a91</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>diag_flash_memory_write_count_exceeded_error</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5fda4931fdb9fc91f5adb48fac5e2ece</anchor>
      <arglist>(bool &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>gyroscope_measurement_range</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a44e9bd65c44b369015681c95dd072229</anchor>
      <arglist>(std::string &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>internal_sensor_bandwidth</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1bb53bfa102423d7f9797534212ecfe9</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_internal_sensor_bandwidth</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aed9c9786bcc89abbc66166c296808682</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>point_of_percussion_alignment</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae35f53220c836a6b902b1af3e6b29350</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_point_of_percussion_alignment</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae05f9c9588cff7f16f86791f1719365a</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>linear_acceleration_compensation</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a62d5175896cbb37ceb6c2658e4b81ea0</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_linear_acceleration_compensation</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa387c9c3dabf8fa37e804c28d00e35d1</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>bias_correction_time_base_control</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ac7bbff84095a3aef133c2e320216e5eb</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_bias_correction_time_base_control</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8c99ce2ea68fd85af6d02db1e1ff65b1</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>x_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a2dbda34c91a4bc5e66e4c3c1c6fa3f4b</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_x_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a590e1c671011e96e5e3f2aec53fa3e09</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>y_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a93d504e8557cb86abdd0be020d32e608</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_y_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1a9f8b34a3ccfa6e00e932d5ab4c8af1</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>z_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af46ebc82a3b4a5953ff99c0096cf670b</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_z_axis_gyroscope_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa2fa9bd24ebef35c9b86bd719b58890e</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>x_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa3e97962189ec360b41dad7ecb85b98c</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_x_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5e307c67ed3955a88515c71977bebadf</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>y_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a486919fad39f28c3bf0302a42afeaaff</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_y_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a854f6da48a9b0eaa16ae6091d757f485</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>z_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab8c092c4c84ea318e588103f1ccb06c7</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>update_z_axis_accelerometer_bias_correction_enable</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa69d01ffb884402a1d9e4d3f3a8e5a1b</anchor>
      <arglist>(uint32_t val)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>bias_correction_update</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8570be1bcfc6e406db889356df69d39a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>factory_calibration_restore</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a52ebe692aa17e496b67fb02244a53852</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>sensor_self_test</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8b9a4a1e97aff28bce9693235474ab42</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>flash_memory_update</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae7e71a1567c6cdfeaf78d39a31fa43cb</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>flash_memory_test</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a415958c4738db5779eb562db27f5d59b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>software_reset</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1ff778a8a82c490d617743be4e68e032</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>firmware_revision</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a99dfab008fa5618c5ddbc2fd66725ee1</anchor>
      <arglist>(std::string &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>firmware_date</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa964105ded1aee0307bae2b9d67c7496</anchor>
      <arglist>(std::string &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>product_id</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5d978a738046b4dd971e4427ce28e8a4</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>serial_number</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a966fe86e585fa6476fad1d076089d216</anchor>
      <arglist>(uint32_t &amp;result)</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>flash_counter</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a2fa5982358d2c391f8126a711c4dbd8a</anchor>
      <arglist>(uint32_t &amp;value)</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_accel</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a72ec08bf510bb536e73bae2e978250a9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_anglvel</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7e3d2c234e0e3448376c99d629e27e15</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_deltavelocity</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a73d5f7d40ed2064828769c67687ff191</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_deltaangl</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab1275be989b72da2776adb0bfa7cc46e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>get_scale_temp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a6da39b748a2bddcecc399cb48db17ab3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>setDeltaAngleScales</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0172adc8d45a187d7a61b99603a3d91b</anchor>
      <arglist>(adis_device_id dev_id)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>setDeltaVelocityScales</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>adb96d076e0d6d85b07d696cb61f092f7</anchor>
      <arglist>(adis_device_id dev_id)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>updateField</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae4b0b02acb4497d4e025f83a84f42266</anchor>
      <arglist>(uint32_t reg, uint32_t val, uint32_t mask)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaAngleXFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a73acc52a805b31ac9c3ba0c59da6fb35</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaAngleYFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8c9bbdecd3b6dc34b52d851ad3dba864</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaAngleZFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0136b1cee75b7d04387340cd484945cd</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaVelocityXFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7b7e6674673e7c8dd5470196c845f45d</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaVelocityYFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab800b398e8b77b5c71a8930ec8080c26</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getRawDeltaVelocityZFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af7d3c2f1e35b7594aa765842b0371caf</anchor>
      <arglist>(int32_t &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaAngleXFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a9c962a79dbb9f6b63906847d7e2cbbb1</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaAngleYFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aff6b2b733f877979f12c807eb221ac32</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaAngleZFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aec63b7635d3c61862cd81ff688d6af02</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaVelocityXFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a3ce5956ca6aa7b57cae3bb391c8ca191</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaVelocityYFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa0d2c7c7369cbabccd4f29b6fe30b568</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="function" protection="private">
      <type>bool</type>
      <name>getConvertedDeltaVelocityZFromDebug</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ac5d005783d54ebe6cebe4bc91ef566b3</anchor>
      <arglist>(double &amp;result)</arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_context *</type>
      <name>m_iio_context</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a443f81cd816e3bbf887db2f26c419c97</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_device *</type>
      <name>m_dev</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa0f6c1e70210dd5cc976e6a8f59ce94f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_device *</type>
      <name>m_dev_trigger</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aac069f57df23f1c6fbe711352388d887</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_buffer *</type>
      <name>m_dev_buffer</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5c3b0fac80d8a6041b4fe74da2f22d28</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_accel_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a3263d648a25afcb89360860aaeaeae83</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_accel_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a30cff563112ab281a3e16479f613913a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_accel_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5e36cfedcf85b8f86de68e553a34b04d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_anglvel_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa26cb1cebb8bac24374124a7b9c5c60f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_anglvel_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a84fdbc7077e2d13efeec0705269d15cd</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_anglvel_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8c440de93f0ddb274f084ad57e6da006</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltaangl_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a70c2b1f9f4c77df8c08d765ba6c3391e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltaangl_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ad7d1984b654f77badbeb5b79996fa70a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltaangl_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a519ef1d68e5bf117ae2184138e78e06c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltavelocity_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a1497cb5175f46cda64be9a65cdae3a74</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltavelocity_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a29952fdee62f9010802a75797cbd76af</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_deltavelocity_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a20e996362ec5b66e2878b6eb73bb4cb7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_temp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0516407c04345ef732f2f66c7ef81daf</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static struct iio_channel *</type>
      <name>m_channel_timestamp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a8725b91265b45a82eb625739c6ae60a1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_accel_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af6c5e7187658b6a02b9f0aba1b5f0513</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_accel_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab046db0a838102e1e6511b7548beff99</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_accel_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a5df3fcc4c271c3b5eb94d590fd03fc6d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_anglvel_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0514cbf50e7afc99c03da258f7a8a0c6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_anglvel_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a4348d695468cd1ce5492a6188c566173</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_anglvel_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7ebe0957beb873e675c8286db05ed5c0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltaangl_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a38f95eabd0320ac8824302716b375004</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltaangl_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a3728986ef27470257106d16a6d22b951</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltaangl_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a4fb8fb22daaa0bdbfdf394016f2706cb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltavelocity_x</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ab3fe5d8585de11cf211b4b6f54a82511</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltavelocity_y</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a0ab2f8825140a4620d2772657427d5e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_deltavelocity_z</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a24fc3690fd18014b7f855beb959aeebc</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>m_scale_temp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>adf5f009168b806991321c9c5ba6bbe48</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static long long</type>
      <name>m_offset_temp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>aa2fef2dbc648e85fb757b7e5fc58f09a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static std::shared_ptr&lt; ADISRegisterMap &gt;</type>
      <name>m_device_descriptor</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a7104c535b1f35bb965ecae6e972e223f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static uint32_t</type>
      <name>buff_write_idx</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a29575e5e1d5af6574ba2f738bb36fa07</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static uint32_t</type>
      <name>buff_read_idx</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a14c3d692d79bb303fcf3ee78e48ad51f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static std::vector&lt; std::vector&lt; uint32_t &gt; &gt;</type>
      <name>buff_data</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>a42dcdfd04ce7a72a87c69734c5ffe8d8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static double</type>
      <name>samp_freq</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>ae20fce83537fd9e9b4f6663bf002fa52</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static uint32_t</type>
      <name>no_of_samp</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af9f2991ff449ed6d9a202a549431311d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private" static="yes">
      <type>static uint32_t</type>
      <name>current_data_selection</name>
      <anchorfile>classadi__imu_1_1IIOWrapper.html</anchorfile>
      <anchor>af8d30b2dc40951a575a610abd9b42a28</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuControlParameters</name>
    <filename>classadi__imu_1_1ImuControlParameters.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuControlParameters</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a61a05aada69aa51ef4eef9f7d1fa0d84</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node, std::shared_ptr&lt; ADISRegisterMap &gt; &amp;device_descriptor)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuControlParameters</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ae31bb4515b8454f1166f1bda21c382d8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>handleControlParams</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a005b70e576aedfa6456e3fbdcbbcaaed</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>UpdateUint32Params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>af246ea4ef6d8ae5d3e02ab12ac05b0c0</anchor>
      <arglist>)(uint32_t)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, UpdateUint32Params &gt;</type>
      <name>UpdateUint32ParamsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a1675ca31f87a2a861c3aa4ec4c47c66c</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>GetUint32Params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aa74819245d4fa96be1f2c8dec49af129</anchor>
      <arglist>)(uint32_t &amp;)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, GetUint32Params &gt;</type>
      <name>GetUint32ParamsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a2bc1073e6f3808bbe604f52b0dfdfbab</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>UpdateInt32Params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a7f0fbef7c4682f175c94465b0b5156d9</anchor>
      <arglist>)(int32_t)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, UpdateInt32Params &gt;</type>
      <name>UpdateInt32ParamsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a7e55d4d5e5e014ab81e50b072f0cf6eb</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>GetInt32Params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>acdabf97cc14b21a841e0e3901ff3a07a</anchor>
      <arglist>)(int32_t &amp;)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, GetInt32Params &gt;</type>
      <name>GetInt32ParamsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a61400317173d0b47dbf7e496a2cfda6e</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>UpdateDoubleParams</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a6b52d203c6932cdd6f2b456d950f53e5</anchor>
      <arglist>)(double)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, UpdateDoubleParams &gt;</type>
      <name>UpdateDoubleParamsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a21e60085a0660c313802edd9a5c1c924</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>GetDoubleParams</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ad4a5f2b09130ddab16c11e8b52c62371</anchor>
      <arglist>)(double *)</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, GetDoubleParams &gt;</type>
      <name>GetDoubleParamsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aa3fb7b4ec118ef25dcc1a6403d2013b0</anchor>
      <arglist></arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>bool(IIOWrapper::*</type>
      <name>ExecuteCommands</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a3e04b37f2ba056cebdb8ff28e5821d21</anchor>
      <arglist>)()</arglist>
    </member>
    <member kind="typedef" protection="private">
      <type>std::map&lt; std::string, ExecuteCommands &gt;</type>
      <name>ExecuteCommandsMapType</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a1ae0b154d7b1b9310a76ab49ad6c2de5</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>declareAdisAttributes</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a33438f1f5764543949f6bc20cc5f8242</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOUpdateFunctionsInt32</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a2e3348742ff830159f7f6ce2254c868e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOGetFunctionsInt32</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a94ea22a29a23b00010e6c0f8a49fde1f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOUpdateFunctionsUint32</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a5539f795b1dbaaf9834b3c03644a7b64</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOGetFunctionsUint32</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a48efa853eaae5653101bcf0b84d330c0</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOUpdateFunctionsDouble</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a13332fb6677ae1a6d26207fae386b234</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOGetFunctionsDouble</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aa4cf5c2478ab7934d5c08e45cb665a98</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>mapIIOCommandFunctions</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ae95a27b60914a9d667232372d6a7e30d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>declareParameterDescription</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aa93dba6c81ad8df7c8713a0ce3084f87</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>declareParameters</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a024895b8912bd1493c247a101cd9c218</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>updateParamsFromHardware</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>af6b48509c7291dd7bfddceb7e6b76db4</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleDoubleParamChange</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a7a9e4c3e1caa28aa60cdeffc13c56626</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleInt32ParamChange</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a1dcfed93e7d94cc4bd8c04a1a65a3d3a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleUint32ParamChange</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ad0c01f8a86dc400b6f339b67241ebf97</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>handleCommands</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>af0d5a5e38d972582fef16a8d9aa71f49</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>affcb8453ef0bdce2e6d79729a97210f8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a066860ee4e43de4fc6f777d7aa84bdf1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::string</type>
      <name>m_command_to_execute</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a50a8010d4dc351c557066dfbf75181f5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>UpdateUint32ParamsMapType</type>
      <name>m_func_map_update_uint32_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aca7d3ffb8f93f545ee1d1fc5397c759d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>GetUint32ParamsMapType</type>
      <name>m_func_map_get_uint32_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aa8032262dfd78edadeda924a419d75e8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>UpdateInt32ParamsMapType</type>
      <name>m_func_map_update_int32_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ae91c61e5e3448e40b4e41507bebc0bd7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>GetInt32ParamsMapType</type>
      <name>m_func_map_get_int32_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ae3cad00755de284847f2897e79f30b55</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>UpdateDoubleParamsMapType</type>
      <name>m_func_map_update_double_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a8ef6472abdcc364fdd688a8cefc9e932</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>GetDoubleParamsMapType</type>
      <name>m_func_map_get_double_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ab6f786d9ca796e7e6e86789906b8eb61</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ExecuteCommandsMapType</type>
      <name>m_func_map_execute_commands</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a07447b24a00aa2a1904e3a3247d49f54</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::list&lt; std::string &gt;</type>
      <name>m_attr_current_device</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>aedd65c4302f22cc38c7f6b94740b957f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, int32_t &gt;</type>
      <name>m_int32_current_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a30cc1b8beffc058a4d9dda5ca9b95831</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, int64_t &gt;</type>
      <name>m_uint32_current_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a7a3b355fa22cada87104a0afcbadc98f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, double &gt;</type>
      <name>m_double_current_params</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>acf4d6ec9b8d0c858f630ad1e5ca83b3d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, std::string &gt;</type>
      <name>m_param_description</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a6b4daccf9290e4d0f7184dcf099722d6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, rcl_interfaces::msg::IntegerRange &gt;</type>
      <name>m_param_constraints_integer</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>ab7d5d470015ddfaae3665ee6cb91465e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::map&lt; std::string, rcl_interfaces::msg::FloatingPointRange &gt;</type>
      <name>m_param_constraints_floating</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>a835fa3947f67357bb01c4c1310e1e515</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::shared_ptr&lt; ADISRegisterMap &gt;</type>
      <name>m_device_descriptor</name>
      <anchorfile>classadi__imu_1_1ImuControlParameters.html</anchorfile>
      <anchor>af3d6467d4d34259be69ddb4d78407bf2</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDataProvider</name>
    <filename>classadi__imu_1_1ImuDataProvider.html</filename>
    <base>adi_imu::ImuDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuDataProvider.html</anchorfile>
      <anchor>ab59782fa5f2c5ac4abd22b1942310d6f</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuDataProvider.html</anchorfile>
      <anchor>a55a80d1e5b5f27e9f521e6f52010d4a5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuDataProvider.html</anchorfile>
      <anchor>a97d76266acb17424247e5b6bf2a32c7a</anchor>
      <arglist>(sensor_msgs::msg::Imu &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1ImuDataProvider.html</anchorfile>
      <anchor>acebed0a2e0d46dfb9bdf830faa6342ba</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDataProviderInterface</name>
    <filename>classadi__imu_1_1ImuDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuDataProviderInterface.html</anchorfile>
      <anchor>a6ff58f5752a6f6bcf3d2f5272e0daa7c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuDataProviderInterface.html</anchorfile>
      <anchor>af82743bfb0238c8739d8168f3966f97e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuDataProviderInterface.html</anchorfile>
      <anchor>a00c66b5fcacd6e661465c6fc2126c343</anchor>
      <arglist>(sensor_msgs::msg::Imu &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDiagDataProvider</name>
    <filename>classadi__imu_1_1ImuDiagDataProvider.html</filename>
    <base>adi_imu::ImuDiagDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuDiagDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProvider.html</anchorfile>
      <anchor>a52c913e911ebb3c696b4d4ed50480ecc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuDiagDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProvider.html</anchorfile>
      <anchor>a0309749e878a7c8673e51effe58cde60</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProvider.html</anchorfile>
      <anchor>a6c47e14c646e62efb74d571baae0c9d0</anchor>
      <arglist>(adi_imu::msg::ImuDiagDataADIS1646X &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProvider.html</anchorfile>
      <anchor>aab67309b2d57202c1efc8ffe936d43f3</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDiagDataProviderInterface</name>
    <filename>classadi__imu_1_1ImuDiagDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuDiagDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProviderInterface.html</anchorfile>
      <anchor>a02ffe604bc2876129d87da90580e7503</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuDiagDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProviderInterface.html</anchorfile>
      <anchor>a00982d8451ec92202e8a248069b6cbcd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuDiagDataProviderInterface.html</anchorfile>
      <anchor>ab0cbc77a9139a3ba30364927d3b3995d</anchor>
      <arglist>(adi_imu::msg::ImuDiagDataADIS1646X &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDiagPublisherFactory</name>
    <filename>classadi__imu_1_1ImuDiagPublisherFactory.html</filename>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDiagRosPublisher</name>
    <filename>classadi__imu_1_1ImuDiagRosPublisher.html</filename>
    <templarg>typename DiagMsgType</templarg>
    <base>adi_imu::ImuDiagRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuDiagRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>adcc4cff1a692c803570f45eeee603bb2</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuDiagRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>a7726b7d82a177c3f9ffc2aa285b6c976</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>a8189bd2e6eca85cdc9a6e657c45a1851</anchor>
      <arglist>(ImuDiagDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setDeviceDescriptor</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>ad420bc72e0b9b558b28ba0d4db22b55a</anchor>
      <arglist>(std::shared_ptr&lt; ADISRegisterMap &gt; device_descriptor) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>a7a316507121ad77cf66b5b8cd63cd6ce</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuDiagDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>a39d6e60a057bed0f0f91fcc8ac0a5f92</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; DiagMsgType &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>acb58091aa9533ae6ae709e952d898c5f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>DiagMsgType</type>
      <name>m_message</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisher.html</anchorfile>
      <anchor>a912ce1f1442866b99be10b72891a0641</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuDiagRosPublisherInterface</name>
    <filename>classadi__imu_1_1ImuDiagRosPublisherInterface.html</filename>
    <base>adi_imu::RosTask</base>
    <member kind="function">
      <type></type>
      <name>ImuDiagRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a04b193e20a1b33f37ea0eed12353010e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuDiagRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a59c72acc0f7b2dd32afc0a0106ebd908</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a499545e0835f5a6699386272f73d5760</anchor>
      <arglist>(ImuDiagDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setDeviceDescriptor</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>adfb4570c9eddbc0a4eea8e20da1908be</anchor>
      <arglist>(std::shared_ptr&lt; ADISRegisterMap &gt; device_descriptor)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1ImuDiagRosPublisherInterface.html</anchorfile>
      <anchor>a9e6cd4f4bfa7cb1f54b94d79e7ca2901</anchor>
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
    <name>adi_imu::ImuFullMeasuredDataProvider</name>
    <filename>classadi__imu_1_1ImuFullMeasuredDataProvider.html</filename>
    <base>adi_imu::ImuFullMeasuredDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a83979542ecbb766bdd9d11261dd96f20</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuFullMeasuredDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>ab57d146c8e40d566875e2f4bd5bb962a</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a738bf13a5794569b4826ec195b7022aa</anchor>
      <arglist>(adi_imu::msg::ImuFullMeasuredData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProvider.html</anchorfile>
      <anchor>a35372b18fd2b1e258241683dbc4d2f53</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuFullMeasuredDataProviderInterface</name>
    <filename>classadi__imu_1_1ImuFullMeasuredDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProviderInterface.html</anchorfile>
      <anchor>a031d07fcfdca4462d4cb790b1bbc9ec8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuFullMeasuredDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProviderInterface.html</anchorfile>
      <anchor>a8a5d013b41b5f155b570d3ed64e4e19b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataProviderInterface.html</anchorfile>
      <anchor>a5b207286b0c3b15c4543a7e816681a00</anchor>
      <arglist>(adi_imu::msg::ImuFullMeasuredData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuFullMeasuredDataRosPublisher</name>
    <filename>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</filename>
    <base>adi_imu::ImuFullMeasuredDataRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>aa603c0e2326496037367ee33a5e451c1</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>abc6d2648f16f83b532e1e6b77ecb6e20</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a740c69557e0fabeab49466d02971514f</anchor>
      <arglist>(ImuFullMeasuredDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a30292e2609a200f589dbd8de7b063af0</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuFullMeasuredDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>abf9b387fa11ea23bba8f1db3e48530ea</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::ImuFullMeasuredData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>a17e5961f9b45be83af73f64f8a9b5f7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::ImuFullMeasuredData</type>
      <name>m_message</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisher.html</anchorfile>
      <anchor>aeae4a64c212566afe2c4e48a69330e10</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuFullMeasuredDataRosPublisherInterface</name>
    <filename>classadi__imu_1_1ImuFullMeasuredDataRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuFullMeasuredDataRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>afa113ed2f8e2550bbd75b665418eb4c5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuFullMeasuredDataRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>ac3b110929d01b501546162887a58d0fd</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>a17f6f20de3a88ceb8ef9ffd41cc5186a</anchor>
      <arglist>(ImuFullMeasuredDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>a31910b4272b89724cb1e857ee8501e34</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1ImuFullMeasuredDataRosPublisherInterface.html</anchorfile>
      <anchor>a501b4366c871fc5fdac4da69e2a0b9dd</anchor>
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
    <name>adi_imu::ImuIdentificationDataProvider</name>
    <filename>classadi__imu_1_1ImuIdentificationDataProvider.html</filename>
    <base>adi_imu::ImuIdentificationDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProvider.html</anchorfile>
      <anchor>acdfb512789a4a762be6cb48fc2f9d808</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuIdentificationDataProvider</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProvider.html</anchorfile>
      <anchor>ada3f594e7451a4caa51704cb4a6b0094</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProvider.html</anchorfile>
      <anchor>ac0e4f7b3bccdaa7a0362b973607c99ac</anchor>
      <arglist>(adi_imu::msg::ImuIdentificationData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProvider.html</anchorfile>
      <anchor>aaf3c260e1ff47380a955abc8a061e5d7</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuIdentificationDataProviderInterface</name>
    <filename>classadi__imu_1_1ImuIdentificationDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProviderInterface.html</anchorfile>
      <anchor>a56aab30deeeaa98b75cc7b3a47b9a5c7</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuIdentificationDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProviderInterface.html</anchorfile>
      <anchor>a705192a05ab2b16e491ce4874e99d44b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationDataProviderInterface.html</anchorfile>
      <anchor>acdac3e097ea00aa1fdab03f016f58f83</anchor>
      <arglist>(adi_imu::msg::ImuIdentificationData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuIdentificationRosPublisher</name>
    <filename>classadi__imu_1_1ImuIdentificationRosPublisher.html</filename>
    <base>adi_imu::ImuIdentificationRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a22406ab0959b7802b09647aac3594af1</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuIdentificationRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>aff693db096cba26fbaab58c39275754b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>ac19b157c0dbeda4ff01054076d1e06da</anchor>
      <arglist>(ImuIdentificationDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>af32c1b3d150daab45ac6672f8d75c9e1</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuIdentificationDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>ae50c3e80633330a289b79ca89f98487f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::ImuIdentificationData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a1392f5657548758678cd3182efebbe1f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::ImuIdentificationData</type>
      <name>m_message</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisher.html</anchorfile>
      <anchor>a8299508a3610b625987a75c7ef654c0c</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuIdentificationRosPublisherInterface</name>
    <filename>classadi__imu_1_1ImuIdentificationRosPublisherInterface.html</filename>
    <base>adi_imu::RosTask</base>
    <member kind="function">
      <type></type>
      <name>ImuIdentificationRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>ac7630f8094ef600488f22448da251204</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuIdentificationRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>abc431d1c879e37da1184f86207de322b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>a6af884f8c710072375eeb84bf467c11d</anchor>
      <arglist>(ImuIdentificationDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1ImuIdentificationRosPublisherInterface.html</anchorfile>
      <anchor>a578ed20564a05adc96f2a6bfbfd6fcf2</anchor>
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
    <name>adi_imu::ImuRosPublisher</name>
    <filename>classadi__imu_1_1ImuRosPublisher.html</filename>
    <base>adi_imu::ImuRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>ImuRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a85c7d89db9a57fe00cdcf32670994246</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~ImuRosPublisher</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a56c48afa8b31989d8b02512bd527e1f8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a86f77f9f4634237ffeb448a91d556120</anchor>
      <arglist>(ImuDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a1a9a8a2bde178043b14520425e7d316b</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a0e2bf559fb79012dcef9a1107731cd10</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; sensor_msgs::msg::Imu &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a7c53f7611e63aeca53fdda13e9eb0c67</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>sensor_msgs::msg::Imu</type>
      <name>m_message</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisher.html</anchorfile>
      <anchor>a9467605700a4cea63b58b0b747062df9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::ImuRosPublisherInterface</name>
    <filename>classadi__imu_1_1ImuRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>ImuRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisherInterface.html</anchorfile>
      <anchor>a20d877a91d58a0573247891ade1f8b4d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~ImuRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisherInterface.html</anchorfile>
      <anchor>aca04157e5baaf56dd0f56ca7a6f1110c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisherInterface.html</anchorfile>
      <anchor>a2f40699e59cd1e07ba941474066a68b8</anchor>
      <arglist>(ImuDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisherInterface.html</anchorfile>
      <anchor>af371071472721923242e715d2800d3bf</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1ImuRosPublisherInterface.html</anchorfile>
      <anchor>a9d9a4a9d4aac14eefbc08b15eafff80e</anchor>
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
    <name>adi_imu::RosPublisherGroup</name>
    <filename>classadi__imu_1_1RosPublisherGroup.html</filename>
    <base>adi_imu::RosPublisherGroupInterface</base>
    <member kind="function">
      <type></type>
      <name>RosPublisherGroup</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>abc68b4fd204a1d2d35aeaa559322be81</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~RosPublisherGroup</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>aad749ded6ca50ed1b28921e31058167b</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setAccelGyroTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a24478a5865a28b992fa7d4cbaf20e2b8</anchor>
      <arglist>(AccelGyroTempRosPublisherInterface *accelGyroTempRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setVelAngTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a149ea8a81129b5f5a44bc1290555a212</anchor>
      <arglist>(VelAngTempRosPublisherInterface *velAngTempRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImuRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a1ba5fdbdb53e9d9f604e8718949d3e6f</anchor>
      <arglist>(ImuRosPublisherInterface *imuRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>ae8a88f04c2611c70fb16cbfca23933bd</anchor>
      <arglist>(ImuFullMeasuredDataRosPublisherInterface *imuFullMeasuredDataRosPublisher) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setImuControlParameters</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a17d4a99ac21d09fe31c9acd54927a217</anchor>
      <arglist>(ImuControlParameters *imuControlParameters) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>run</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a061ea62ae21d14c58d2deb4da2aed10e</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>AccelGyroTempRosPublisherInterface *</type>
      <name>m_accelGyroTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a1e8390831ef3806eb82459847bb51517</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>VelAngTempRosPublisherInterface *</type>
      <name>m_velAngTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a5a1763a78e806d718a1a34ae08b12ead</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuRosPublisherInterface *</type>
      <name>m_imuRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a17190ae9754c04e1690db19b159c5cfb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuFullMeasuredDataRosPublisherInterface *</type>
      <name>m_imuFullMeasuredDataRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>ae09989811efe5b5f0bd669d65d19909c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ImuControlParameters *</type>
      <name>m_imuControlParameters</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroup.html</anchorfile>
      <anchor>a027b60e12cea69b1e0b296a27b368947</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::RosPublisherGroupInterface</name>
    <filename>classadi__imu_1_1RosPublisherGroupInterface.html</filename>
    <base>adi_imu::RosTask</base>
    <member kind="function">
      <type></type>
      <name>RosPublisherGroupInterface</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>a2ca919139293ef8f66a261b95754e7ae</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RosPublisherGroupInterface</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>a935b3a511919ed7bf395ef03507fe709</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setAccelGyroTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>a17957c09311092a6e90c11990ae04d0b</anchor>
      <arglist>(AccelGyroTempRosPublisherInterface *accelGyroTempRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setVelAngTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>ae3ec040724937cfe3aeed66af8199528</anchor>
      <arglist>(VelAngTempRosPublisherInterface *velAngTempRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setImuRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>ac65aa4a6e97319c086f9ee67ff584589</anchor>
      <arglist>(ImuRosPublisherInterface *imuRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setImuFullMeasuredDataRosPublisher</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>a3206f5b8fddf38fa2306017bc6a3561c</anchor>
      <arglist>(ImuFullMeasuredDataRosPublisherInterface *imuFullMeasuredDataRosPublisher)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setImuControlParameters</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>a517de488d98bc902b59a7b6382c500f8</anchor>
      <arglist>(ImuControlParameters *imuControlParameters)=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1RosPublisherGroupInterface.html</anchorfile>
      <anchor>a061d21fc0fffa91e419d514d3d3ea374</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::RosTask</name>
    <filename>classadi__imu_1_1RosTask.html</filename>
    <member kind="function">
      <type></type>
      <name>RosTask</name>
      <anchorfile>classadi__imu_1_1RosTask.html</anchorfile>
      <anchor>aabef10d9a1f8ec699ee8b4fdeec8f7fa</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~RosTask</name>
      <anchorfile>classadi__imu_1_1RosTask.html</anchorfile>
      <anchor>ad36570b4d84c20ec267c433c9122c010</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>run</name>
      <anchorfile>classadi__imu_1_1RosTask.html</anchorfile>
      <anchor>a8750563a3003820f2a7da10dc467b055</anchor>
      <arglist>()=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::VelAngTempDataProvider</name>
    <filename>classadi__imu_1_1VelAngTempDataProvider.html</filename>
    <base>adi_imu::VelAngTempDataProviderInterface</base>
    <member kind="function">
      <type></type>
      <name>VelAngTempDataProvider</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProvider.html</anchorfile>
      <anchor>a698cb764427cb7b8969d549b66a7d3d9</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~VelAngTempDataProvider</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProvider.html</anchorfile>
      <anchor>a4afe61106eaa1fd376224a0277db8d76</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProvider.html</anchorfile>
      <anchor>afc9536d9b23029586092177e8b531aad</anchor>
      <arglist>(adi_imu::msg::VelAngTempData &amp;message) override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>IIOWrapper</type>
      <name>m_iio_wrapper</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProvider.html</anchorfile>
      <anchor>a5dd4dec8be1993f0fa81cf3a0b9a9066</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::VelAngTempDataProviderInterface</name>
    <filename>classadi__imu_1_1VelAngTempDataProviderInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>VelAngTempDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProviderInterface.html</anchorfile>
      <anchor>a8c619652e5600d65455384beaf019cc6</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~VelAngTempDataProviderInterface</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProviderInterface.html</anchorfile>
      <anchor>a1fe6c3fbaa4dd935daa6217d5869b995</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual bool</type>
      <name>getData</name>
      <anchorfile>classadi__imu_1_1VelAngTempDataProviderInterface.html</anchorfile>
      <anchor>a8dd4bbddf4f56e1b570798519deac35b</anchor>
      <arglist>(adi_imu::msg::VelAngTempData &amp;message)=0</arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::VelAngTempRosPublisher</name>
    <filename>classadi__imu_1_1VelAngTempRosPublisher.html</filename>
    <base>adi_imu::VelAngTempRosPublisherInterface</base>
    <member kind="function">
      <type></type>
      <name>VelAngTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>a78ca8bddb7d11ffab85519e22e2687c9</anchor>
      <arglist>(std::shared_ptr&lt; rclcpp::Node &gt; &amp;node)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~VelAngTempRosPublisher</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>a3d8e8ec99a942f4a342cd5e4fca8117c</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>aa95e8ba1150f64b0a2d44a519dafca25</anchor>
      <arglist>(VelAngTempDataProviderInterface *dataProvider) override</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>a53b7c1cc3cbc062a7b5a0a32febf3546</anchor>
      <arglist>() override</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>VelAngTempDataProviderInterface *</type>
      <name>m_data_provider</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>a746d90da062a25dac74924ab00152e63</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>rclcpp::Publisher&lt; adi_imu::msg::VelAngTempData &gt;::SharedPtr</type>
      <name>m_publisher</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>a77467118945b751b61c38e0004109a17</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>adi_imu::msg::VelAngTempData</type>
      <name>m_message</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisher.html</anchorfile>
      <anchor>afc9cf345d166f641bc6927711cafa081</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>adi_imu::VelAngTempRosPublisherInterface</name>
    <filename>classadi__imu_1_1VelAngTempRosPublisherInterface.html</filename>
    <member kind="function">
      <type></type>
      <name>VelAngTempRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>a19131c5961639a683ed414a4f2c06224</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="virtual">
      <type>virtual</type>
      <name>~VelAngTempRosPublisherInterface</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>a1adb2af54d5748a9fe99602493539528</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>setMessageProvider</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>ab35ef2daa721d52daf9bc5b0bea982d7</anchor>
      <arglist>(VelAngTempDataProviderInterface *dataProvider)=0</arglist>
    </member>
    <member kind="function" virtualness="pure">
      <type>virtual void</type>
      <name>publish</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>ac5faa37596fc59fdade8eb202b970f64</anchor>
      <arglist>()=0</arglist>
    </member>
    <member kind="variable" protection="protected">
      <type>std::shared_ptr&lt; rclcpp::Node &gt;</type>
      <name>m_node</name>
      <anchorfile>classadi__imu_1_1VelAngTempRosPublisherInterface.html</anchorfile>
      <anchor>af87138dded7510ac264c93a7a333b7da</anchor>
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
    <name>adi_imu::WorkerThread</name>
    <filename>classadi__imu_1_1WorkerThread.html</filename>
    <member kind="function">
      <type></type>
      <name>WorkerThread</name>
      <anchorfile>classadi__imu_1_1WorkerThread.html</anchorfile>
      <anchor>a037ff54439a1503153235a642af66a52</anchor>
      <arglist>(RosTask *rosTask)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~WorkerThread</name>
      <anchorfile>classadi__imu_1_1WorkerThread.html</anchorfile>
      <anchor>a790f9870922446ef30396767bd09eabe</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>runTask</name>
      <anchorfile>classadi__imu_1_1WorkerThread.html</anchorfile>
      <anchor>a0143383e5fa99b1e2b49b408e60c0768</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>RosTask *</type>
      <name>m_rosTask</name>
      <anchorfile>classadi__imu_1_1WorkerThread.html</anchorfile>
      <anchor>a45b31cf39c3820920b0135128390cd12</anchor>
      <arglist></arglist>
    </member>
  </compound>
</tagfile>
