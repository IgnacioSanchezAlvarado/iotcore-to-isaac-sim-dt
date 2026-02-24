import * as cdk from 'aws-cdk-lib';
import * as ec2 from 'aws-cdk-lib/aws-ec2';
import * as iam from 'aws-cdk-lib/aws-iam';
import * as autoscaling from 'aws-cdk-lib/aws-autoscaling';
import * as s3assets from 'aws-cdk-lib/aws-s3-assets';
import { Construct } from 'constructs';
import * as fs from 'fs';
import * as path from 'path';

// Load project configuration
const config = JSON.parse(
  fs.readFileSync(path.join(__dirname, '../../config.json'), 'utf-8')
);

export class IsaacSimDTStack extends cdk.Stack {
  constructor(scope: Construct, id: string, props?: cdk.StackProps) {
    super(scope, id, props);

    // --- VPC: Multiple AZs for GPU capacity flexibility ---
    const vpc = new ec2.Vpc(this, 'Vpc', {
      maxAzs: 3,
      natGateways: 0,
      subnetConfiguration: [
        {
          name: 'Public',
          subnetType: ec2.SubnetType.PUBLIC,
        },
      ],
    });

    // --- Security Group ---
    const sg = new ec2.SecurityGroup(this, 'IsaacSimSG', {
      vpc,
      description: 'Isaac Sim DT - DCV and SSH access',
      allowAllOutbound: true,
    });

    sg.addIngressRule(
      ec2.Peer.ipv4(config.ec2.clientIpCidr),
      ec2.Port.tcp(config.dcv.port),
      'NICE DCV access'
    );

    sg.addIngressRule(
      ec2.Peer.ipv4(config.ec2.clientIpCidr),
      ec2.Port.tcp(22),
      'SSH access'
    );

    // --- IAM Role ---
    const role = new iam.Role(this, 'EC2Role', {
      assumedBy: new iam.ServicePrincipal('ec2.amazonaws.com'),
      managedPolicies: [
        iam.ManagedPolicy.fromAwsManagedPolicyName('AmazonSSMManagedInstanceCore'),
      ],
    });

    // IoT Core: MQTT over WebSocket with SigV4
    role.addToPolicy(
      new iam.PolicyStatement({
        effect: iam.Effect.ALLOW,
        actions: ['iot:Connect'],
        resources: [`arn:aws:iot:*:*:client/isaac-sim-dt-*`],
      })
    );

    role.addToPolicy(
      new iam.PolicyStatement({
        effect: iam.Effect.ALLOW,
        actions: ['iot:Subscribe'],
        resources: [
          `arn:aws:iot:*:*:topicfilter/${config.iot.topicPrefix}/${config.iot.deviceId}/*`,
        ],
      })
    );

    role.addToPolicy(
      new iam.PolicyStatement({
        effect: iam.Effect.ALLOW,
        actions: ['iot:Receive'],
        resources: [
          `arn:aws:iot:*:*:topic/${config.iot.topicPrefix}/${config.iot.deviceId}/*`,
        ],
      })
    );

    role.addToPolicy(
      new iam.PolicyStatement({
        effect: iam.Effect.ALLOW,
        actions: ['iot:DescribeEndpoint'],
        resources: ['*'],
      })
    );

    // CloudWatch Logs
    role.addToPolicy(
      new iam.PolicyStatement({
        effect: iam.Effect.ALLOW,
        actions: [
          'logs:CreateLogGroup',
          'logs:CreateLogStream',
          'logs:PutLogEvents',
        ],
        resources: ['*'],
      })
    );

    // Secrets Manager: DCV password storage
    role.addToPolicy(
      new iam.PolicyStatement({
        effect: iam.Effect.ALLOW,
        actions: ['secretsmanager:CreateSecret', 'secretsmanager:PutSecretValue', 'secretsmanager:UpdateSecret'],
        resources: [`arn:aws:secretsmanager:${cdk.Aws.REGION}:${cdk.Aws.ACCOUNT_ID}:secret:isaac-sim-dt/dcv-password*`],
      })
    );

    // --- S3 Assets for backend code and config ---
    const backendAsset = new s3assets.Asset(this, 'BackendAsset', {
      path: path.join(__dirname, '../../backend'),
      exclude: ['scripts', '__pycache__', '*.pyc'],
    });

    const configAsset = new s3assets.Asset(this, 'ConfigAsset', {
      path: path.join(__dirname, '../../config.json'),
    });

    // Grant EC2 instance read access to assets
    backendAsset.grantRead(role);
    configAsset.grantRead(role);

    // --- UserData: Backend asset download + bootstrap ---
    const userData = ec2.UserData.forLinux();

    // Asset download commands
    userData.addCommands(
      '#!/bin/bash',
      'set -euxo pipefail',
      'exec > /var/log/dt-asset-download.log 2>&1',
      '',
      'echo "=== Downloading backend assets ==="',
      'date',
      '',
      '# Install AWS CLI if not present',
      'if ! command -v aws &>/dev/null; then',
      '  echo "Installing AWS CLI..."',
      '  apt-get update -y && apt-get install -y unzip curl',
      '  curl -sSL "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o /tmp/awscliv2.zip',
      '  unzip -q /tmp/awscliv2.zip -d /tmp',
      '  /tmp/aws/install',
      '  rm -rf /tmp/aws /tmp/awscliv2.zip',
      'fi',
      '',
      '# Create application directory',
      'mkdir -p /opt/digital-twin/backend',
      '',
      '# Download backend code archive',
      `aws s3 cp s3://${backendAsset.s3BucketName}/${backendAsset.s3ObjectKey} /tmp/backend.zip`,
      '',
      '# Extract backend code',
      'cd /opt/digital-twin/backend',
      'unzip -o /tmp/backend.zip',
      'rm /tmp/backend.zip',
      '',
      '# Download config.json',
      `aws s3 cp s3://${configAsset.s3BucketName}/${configAsset.s3ObjectKey} /opt/digital-twin/config.json`,
      '',
      '# Set ownership for service user',
      'DEFAULT_USER=$(getent passwd 1000 | cut -d: -f1 || echo "ubuntu")',
      'chown -R $DEFAULT_USER:$DEFAULT_USER /opt/digital-twin',
      '',
      'echo "=== Backend assets downloaded ==="',
      'date'
    );

    // Bootstrap script: ROS2, Python deps, USD model, DCV, systemd service
    const bootstrapScript = fs.readFileSync(
      path.join(__dirname, '../../backend/scripts/bootstrap.sh'),
      'utf-8'
    );
    userData.addCommands(bootstrapScript);

    // --- Launch Template ---
    // NOTE: Replace AMI IDs in config.json with actual AMI IDs after
    // subscribing to the Isaac Sim marketplace offering.
    // The marketplace product ID (prodview-bl35herdyozhw) must be resolved
    // to a region-specific AMI ID in your account.
    const launchTemplate = new ec2.LaunchTemplate(this, 'IsaacSimLT', {
      instanceType: new ec2.InstanceType(config.ec2.instanceType),
      machineImage: ec2.MachineImage.genericLinux(config.ec2.amiMapping),
      securityGroup: sg,
      role,
      userData,
      associatePublicIpAddress: true,
      blockDevices: [
        {
          deviceName: '/dev/sda1',
          volume: ec2.BlockDeviceVolume.ebs(config.ec2.volumeSizeGb, {
            volumeType: ec2.EbsDeviceVolumeType.GP3,
            encrypted: true,
          }),
        },
      ],
    });

    // --- Auto Scaling Group ---
    const asg = new autoscaling.AutoScalingGroup(this, 'IsaacSimASG', {
      vpc,
      launchTemplate,
      minCapacity: 1,
      maxCapacity: 1,
      desiredCapacity: 1,
      vpcSubnets: { subnetType: ec2.SubnetType.PUBLIC },
    });

    // Tag ASG instances
    cdk.Tags.of(asg).add('Name', 'isaac-sim-dt');

    // --- Tags ---
    cdk.Tags.of(this).add('Project', config.project.name);
    cdk.Tags.of(this).add('Environment', config.project.environment);

    // --- Outputs ---
    new cdk.CfnOutput(this, 'AutoScalingGroupName', {
      value: asg.autoScalingGroupName,
      description: 'Auto Scaling Group name',
    });

    new cdk.CfnOutput(this, 'SSMCommand', {
      value: "Use 'aws ec2 describe-instances --filters Name=tag:Name,Values=isaac-sim-dt Name=instance-state-name,Values=running' or AWS Console to find the instance ID, then: aws ssm start-session --target <INSTANCE_ID>",
      description: 'SSM Session Manager connect command',
    });

    new cdk.CfnOutput(this, 'DCVPasswordSecret', {
      value: 'isaac-sim-dt/dcv-password',
      description: 'Retrieve DCV password: aws secretsmanager get-secret-value --secret-id isaac-sim-dt/dcv-password --query SecretString --output text',
    });

    new cdk.CfnOutput(this, 'DCVUrl', {
      value: 'https://<public-ip>:8443 — find IP with: aws ec2 describe-instances --filters Name=tag:Name,Values=isaac-sim-dt Name=instance-state-name,Values=running --query Reservations[].Instances[].PublicIpAddress --output text',
      description: 'NICE DCV connection URL',
    });
  }
}
